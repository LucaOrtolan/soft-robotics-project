import utils as u
from math import pi, degrees
import pandas as pd
import numpy as np

class Model:

    def __init__(self, model_type: str, material: str):
        """Create mathematical model for a specific material.
        
        Args:
            model_type: NeoHookean, Mooney-Rivlin or Ogden
            material: EcoFlex 0050, Dragon Skin 30 or Sylgard 184"""
        
        self.materials = u.import_json_data("materials")

        self.model_type = model_type
        self.material = material

        if self.model_type not in ["NeoHookean", "Mooney-Rivlin", "Ogden"]:
            print (f"{self.model_type} not supported.")

        try:        
            self.mu = self.materials[self.material]["mu"]
            self.C10 = self.materials[self.material]["C10"]
            self.C01 = self.materials[self.material]["C01"]
            self.alpha = self.materials[self.material]["alpha"]
            self.K = self.materials[self.material]["K"]

        except KeyError:
            print(f"Material: {self.material} not found")
            
    def compute_shear_modulus(self):
        if self.model_type == "NeoHookean":
            self.shear_modulus = self.mu
        elif self.model_type == "Mooney-Rivlin":
            self.shear_modulus = 2*(self.C10+self.C01)  
        elif self.model_type == "Ogden":
            self.shear_modulus = (self.mu*self.alpha)/2
        return self.shear_modulus

    def compute_tangent_modulus(self, pre_strain: float):
        self.stretch_ratio = 1 + pre_strain
        if self.model_type == "NeoHookean":
            self.tangent_modulus = self.mu*(2*(self.stretch_ratio)+(self.stretch_ratio)**-2)
        elif self.model_type == "Mooney-Rivlin":
            self.tangent_modulus = 2*((-self.C01*self.stretch_ratio**-2)*(self.stretch_ratio**2-self.stretch_ratio**-1)+(self.C10+self.C01*self.stretch_ratio**-1)*(2*self.stretch_ratio+self.stretch_ratio**-2))
        elif self.model_type == "Ogden":
            self.tangent_modulus = self.mu*(self.alpha*self.stretch_ratio**(self.alpha-1)+self.alpha/2*self.stretch_ratio**(-self.alpha/2-1))
        return self.tangent_modulus * 10**6 # rescale to Pascals


class PhysicsEngine(Model):

    def __init__(self):
        self.data = u.import_json_data("data")

        self.model_type = self.data["model"]
        self.material = self.data["material"]
        self.pre_strain = self.data["pre_axial_strain"]
        self.pressure_a = self.data["pressure_a"] * 1000 # rescale to Pa
        self.pressure_b = self.data["pressure_b"] * 1000 # rescale to Pa
        self.channel_radius = self.data["channel_radius"]/1000 # rescale to m
        self.septum_thickness = self.data["septum_thickness"]/1000 # rescale to m
        self.robot_data = pd.DataFrame(self.data["robot_data"])

        super().__init__(self.model_type, self.material)
        self.shear_modulus = self.compute_shear_modulus()
        self.tangent_modulus = self.compute_tangent_modulus(self.pre_strain)

    def convert_geometric_variables_to_m(self, data):
        for col in data.columns:
            if "mm" in col:
                data[col] /= 1000
                data = data.rename(columns={col: col.replace("mm", "m")})
        return data

    def compute_structural_rigidity(self):
        df = self.convert_geometric_variables_to_m(self.robot_data)
        df["Inner Radius (m)"] = df["Outer Radius (m)"] - df["Thickness (m)"]
        df["I (m^4)"] = pi/4*(df["Outer Radius (m)"]**4-df["Inner Radius (m)"]**4)

        df["EI (Nm^2)"] = self.tangent_modulus*df["I (m^4)"]
        return df
    
    def compute_actuation(self):
        delta_p = (self.pressure_a-self.pressure_b)
        channel_area = (pi*(self.channel_radius**2))/2
        centroid_distance = (self.channel_radius+self.septum_thickness/2)+(4*pe.channel_radius)/(3*pi)
        moment = delta_p*channel_area*centroid_distance
        return moment
    
    def compute_curvature(self):
        df = self.compute_structural_rigidity()
        df["M (N*m)"] = self.compute_actuation()
        df["k (rad/m)"] = df["M (N*m)"]/df["EI (Nm^2)"]
        df["rho (m)"] = 1/df["k (rad/m)"]
        return df

    def compute_segment_arc_angles(self):
        df = self.compute_curvature()
        df["Length_pre (m)"] = df["Length (m)"]*(self.stretch_ratio)
        df["theta (rad)"] = df["k (rad/m)"] * df["Length_pre (m)"]
        df["theta (deg)"] = df["theta (rad)"].apply(lambda x: degrees(x))
        # self intersection check df["theta (deg)"].sum() < 270
        return df

    def pcc_kinematics(self):
        df = self.compute_segment_arc_angles()

        # Initialize list of homogeneous transforms and positions
        T_total = np.eye(4)

        for _, row in df.iterrows():
            k = row["k (rad/m)"]
            theta = row["theta (rad)"]

            # Constant‑curvature homogeneous transform for planar bending in x–z
            # Webster-style CC transform: bend about local y axis [web:14]
            T_i = np.array([
                [ np.cos(theta),  0.0,  np.sin(theta),  (1.0 - np.cos(theta))/k ],
                [ 0.0,            1.0,  0.0,            0.0                     ],
                [ -np.sin(theta), 0.0,  np.cos(theta),  np.sin(theta)/k         ],
                [ 0.0,            0.0,  0.0,            1.0                     ]
            ])

            # Update global transform
            T_total = T_total @ T_i

        # Extract current segment tip position in base frame
        final_p = T_total[:3, 3]
        final_p = pd.DataFrame(final_p, index=["x", "y", "z"], columns=["Coordinate"])
        return df, final_p
    
    # --------------- WRAPPER FUNCTION ---------------
    def run_analysis(self):
        end = "\n\n"
        df, final_p = self.pcc_kinematics()

        print(f"Final Coordinates: {final_p}")

        print("------- CHECKS ------")
        validation_error = self.check_validation()
        curvatures_check = self.check_curvatures(df)
        self_intersection_check = self.check_self_intersection(df)
        arc_length_check = self.check_arc_length(df)
        segment_tensile_strain, total_strain = self.check_maximum_tensile_strain(df)
        thin_wall_check = self.check_thin_wall_assumption(df)
        final_uncertainty = self.check_final_uncertainty(final_p)

        print("*** Perfect incompressibility validation result = %.4f"%(validation_error), end=end)
        print(f"*** Curvatures magnitude check:\n{curvatures_check}", end=end)
        print("*** Self intersection check = %.4f"%(self_intersection_check), end=end)
        print("*** Total Arc Length - Total Length Pre = %.4f"%(arc_length_check), end=end)
        print(f"*** Segment wise tensile strain:\n {segment_tensile_strain}\n"+
              "** Total tensile strain = %.4f"%(total_strain), end=end)
        print(f"*** Thin-Wall assumption: {thin_wall_check}", end=end)
        print(f"*** Final position uncertainty: {final_uncertainty}", end=end)


    # --------------- CHECKS ---------------
    def check_validation(self):
        """Check to verify the assumption of perfect incompressibility. 
        The assumption holds if error < 2%"""
        e_approx = 3*self.mu
        e_full = (9*self.K*self.mu)/(3*self.K+self.mu)
        error = abs((e_approx-e_full)/e_full)
        return error
    
    def check_curvatures(self, df):
        """Checks order of magnitude for curvatures. 
        For a soft robot, they are usually in a 1-100 rad/m range.
        Greater values may imply too soft segments or too much moment, which could lead to buckling."""
        df["Magnitude in range"] = df["k (rad/m)"].between(1, 100)
        return df[["k (rad/m)", "Magnitude in range"]]
    
    def check_self_intersection(self, df):
        """Checks the sum of the final theta angles. 
        If >270°, there is high risk of self-intersection."""
        return df["theta (deg)"].sum()
    
    # ----------- PHYSICAL PLAUSIBILITY -----------
    def check_arc_length(self, df):
        """The sum of the calculated arc lengths must be equal to the total pre-strained length of each segment.
        It fails if error > 0.1%. Checks for calculations and rounding errors"""
        total_arc_length = sum(df["rho (m)"]*df["theta (rad)"])
        total_length_pre = sum(df["Length_pre (m)"])
        error = (total_arc_length-total_length_pre)/total_length_pre
        return error
    
    def check_maximum_tensile_strain(self, df):
        """Calculates the strain for each segment given as k(i)*R(i) + pre_strain and sum them."""
        df["Segment strain"] = df["k (rad/m)"] * df["Outer Radius (m)"] + self.pre_strain
        maximum_strain = df["Segment strain"].sum()
        return df["Segment strain"], maximum_strain
    
    def check_thin_wall_assumption(self, df):
        """Verifies thin-wall assumption in the beam model segment-wise"""
        df["Thickness/Outer Radius"] = df["Thickness (m)"]/df["Outer Radius (m)"]
        df["Thin-Wall"] = df["Thickness/Outer Radius"] < 0.2
        return df[["Thickness (m)", "Outer Radius (m)", "Thickness/Outer Radius", "Thin-Wall"]]
    
    def check_final_uncertainty(self, final_p, step_uncertainty=0.01):
        """Computes uncertainty interval for the final position vector assuming uncertainty(%)=sqrt(5)*step_uncertainty (by default 1%)"""
        uncertainty = np.sqrt(5)*step_uncertainty
        delta = final_p["Coordinate"]*uncertainty
        final_p["Lower Bound"] = final_p["Coordinate"] - delta
        final_p["Upper Bound"] = final_p["Coordinate"] + delta
        return final_p

    
        


    











pe = PhysicsEngine()

final_p = pe.run_analysis()
    


         

    