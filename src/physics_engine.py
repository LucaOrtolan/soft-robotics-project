import utils as u
from math import pi, degrees
import pandas as pd

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
    
    def validation_check(self):
        e_approx = 3*self.mu
        e_full = (9*self.K*self.mu)/(3*self.K+self.mu)
        error = abs((e_approx-e_full)/e_full) # if <2% the assumption of perfect incompressibility stands
        return error


class PhysicsEngine(Model):

    def __init__(self):
        self.data = u.import_json_data("data")

        self.model_type = self.data["model"]
        self.material = self.data["material"]
        self.pre_strain = self.data["pre_axial_strain"]
        self.pressure_a = self.data["pressure_a"]
        self.pressure_b = self.data["pressure_b"]
        self.channel_radius = self.data["channel_radius"]
        self.septum_thickness = self.data["septum_thickness"]
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
        delta_p = (self.pressure_a-self.pressure_b) * 1000 # rescale to Pa
        channel_radius_m = self.channel_radius/1000 # rescale to m
        channel_area = (pi*channel_radius_m**2)/2
        centroid_distance = (channel_radius_m+self.septum_thickness/2)+(4*channel_radius_m)/(3*pi)
        moment = delta_p*channel_area*centroid_distance
        return moment
    
    def compute_curvature(self):
        df = self.compute_structural_rigidity()
        df["M (N*m)"] = self.compute_actuation()
        df["k (rad/m)"] = df["M (N*m)"]/df["EI (Nm^2)"]
        df["rho (m)"] = 1/df["k (rad/m)"]
        return df

    def compute_ppc_kinematics(self):
        df = self.compute_curvature()
        df["Length_pre (m)"] = df["Length (m)"]*(self.stretch_ratio)
        df["theta (rad)"] = df["k (rad/m)"] * df["Length_pre (m)"]
        df["theta (deg)"] = df["theta (rad)"].apply(lambda x: degrees(x))
        return df
    
    # def compute_tip_position(self):
    #     df = self.compute_pcc_kinematics()


pe = PhysicsEngine()

df = pe.compute_ppc_kinematics()
print(df)
    


         

    