import streamlit as st
import pandas as pd
import utils as u

materials = u.import_json_data("materials")

if st.session_state.get("data") is None:
    st.session_state={
        "data":{
            "n_segments": 1,
            "material": None,
            "model": None,
            "pre_axial_strain": 0,
            "pressure_a": 0,
            "pressure_b": 0,
            "channel_radius": 0,
            "septum_thickness": 0,
            "robot_data": None,
            "inverse_kinematics": False,
            "target_p": None
            }
    }

with st.form("Declare Parameters"):
    st.markdown("#### Parameters")
    st.session_state["data"]["n_segments"] = st.number_input("Number of segments", min_value=1, step=1, placeholder=3)
    st.session_state["data"]["model"] = st.selectbox("Model", ["NeoHookean", "Mooney-Rivlin", "Ogden"])
    st.session_state["data"]["pre_axial_strain"] = st.number_input("Pre axial strain", min_value=0.0, step=0.01)
    st.session_state["data"]["pressure_a"] = st.number_input("Pressure A (kPa)", min_value=1, step=1, placeholder=10)
    st.session_state["data"]["pressure_b"] = st.number_input("Pressure B (kPa)", min_value=1, step=1, placeholder=10)
    st.session_state["data"]["channel_radius"] = st.number_input("Channel radius (mm)", min_value=1, step=1, placeholder=5.0)
    st.session_state["data"]["septum_thickness"] = st.number_input("Septum thickness (mm)", min_value=0.1, step=0.1, placeholder=0.8)
    st.session_state["data"]["material"] = st.selectbox("Material", materials.keys())

    st.form_submit_button("Confirm Parameters")

st.session_state["data"]["inverse_kinematics"] = st.checkbox("Run Inverse Kinematics")
if st.session_state["data"]["inverse_kinematics"]:
    with st.container():
        st.markdown("#### Specify Target Position")
        st.session_state["data"]["target_p"] = st.data_editor(pd.DataFrame({"Coordinate": [0.0, 0.0]}, index=["x", "z"]), width="content")


robot_df = pd.DataFrame({
    "Length (mm)": [0.0] * st.session_state["data"]["n_segments"],
    "Outer Radius (mm)": [0.0] * st.session_state["data"]["n_segments"],
    "Thickness (mm)": [0.0] * st.session_state["data"]["n_segments"]
})
robot_df.index.name = "Segment(i)"

st.write("Insert Segment Data")
st.session_state["data"]["robot_data"] = st.data_editor(robot_df)

if st.button("Run Simulation"):
    st.session_state["data"]["robot_data"] = st.session_state["data"]["robot_data"].to_dict()
    st.session_state["data"]["target_position"] = st.session_state["data"]["target_position"].to_dict()
    u.export_json(st.session_state["data"], "data")
    st.write("data successfully exported")