import json
import os

this_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(this_dir, ".."))
jsons_dir = os.path.join(parent_dir, "jsons")

class Model:

    def __init__(self, model_type: str, material: str):
        """Create mathematical model for a specific material.
        
        Args:
            model_type: NeoHookean, Mooney-Rivlin or Ogden
            material: EcoFlex 0050, Dragon Skin 30 or Sylgard 184"""
        
        with open(os.path.join(jsons_dir, "materials.json"), "r") as f:
            self.materials = json.load(f)

        self.model_type = model_type
        self.material = material

        if self.model_type not in ["NeoHookean", "Mooney-Rivlin", "Ogden"]:
            print (f"{self.model_type} not supported.")

        if self.material not in list(self.materials.keys()):
            print(f"{self.material} not supported")
        
        self.mu = self.materials[self.material]["mu"]
        self.C10 = self.materials[self.material]["C10"]
        self.C01 = self.materials[self.material]["C01"]
        self.alpha = self.materials[self.material]["alpha"]
        self.K = self.materials[self.material]["K"]

    def compute_shear_modulus(self):
        if self.model_type == "NeoHookean":
            self.shear_modulus = self.mu
        elif self.model_type == "Mooney-Rivlin":
            self.shear_modulus = 2*(self.C10+self.C01)  
        elif self.model_type == "Ogden":
            self.shear_modulus = (self.mu*self.alpha)/2
        return self.shear_modulus

    def compute_tangent_modulus(self, pre_strain: float):
        stretch_ratio = 1 + pre_strain
        if self.model_type == "NeoHookean":
            self.tangent_modulus = self.mu*(2*(stretch_ratio)+(stretch_ratio)**-2)
        elif self.model_type == "Mooney-Rivlin":
            self.tangent_modulus = 2*((-self.C01*stretch_ratio**-2)*(stretch_ratio**2-stretch_ratio**-1)+(self.C10+self.C01*stretch_ratio**-1)*(2*stretch_ratio+stretch_ratio**-2))
        elif self.model_type == "Ogden":
            self.tangent_modulus = self.mu*(self.alpha*stretch_ratio**(self.alpha-1)+self.alpha/2*stretch_ratio**(-self.alpha/2-1))
        return self.tangent_modulus
    
    def validation_check(self):
        e_approx = 3*self.mu
        e_full = (9*self.K*self.mu)/(3*self.K+self.mu)
        error = abs((e_approx-e_full)/e_full) # if <2% the assumption of perfect incompressibility stands
        return error

    