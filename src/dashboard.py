import streamlit as st
import pandas as pd
import utils as u

materials = u.import_materials_data()

if st.session_state is None:
    st.session_state={
        "n_segments": 1,
        "material": None,
        "pre_axial_strain": 0,
        "pressure_a": 0,
        "pressure_b": 0,
        "channel_radius": 0,
        "septum_thickness": 0,
        "robot_data": None
    }

with st.form("Declare Parameters"):
    st.session_state["n_segments"] = st.number_input("Number of segments", min_value=1, step=1, placeholder=3)
    st.session_state["pre_axial_strain"] = st.number_input("Pre axial strain", min_value=0.0, step=0.01)
    st.session_state["pressure_a"] = st.number_input("Pressure A (kPa)", min_value=1, step=1, placeholder=10)
    st.session_state["pressure_b"] = st.number_input("Pressure B (kPa)", min_value=1, step=1, placeholder=10)
    st.session_state["channel_radius"] = st.number_input("Channel radius (mm)", min_value=1, step=1, placeholder=5.0)
    st.session_state["septum_thickness"] = st.number_input("Septum thickness (mm)", min_value=0.1, step=0.1, placeholder=0.8)
    st.session_state["material"] = st.selectbox("Material", materials.keys())

    st.form_submit_button("Confirm Parameters")

robot_df = pd.DataFrame({
    "Length (mm)": [0.0] * st.session_state["n_segments"],
    "Outer Radius (mm)": [0.0] * st.session_state["n_segments"],
    "Thickness (mm)": [0.0] * st.session_state["n_segments"]
})
robot_df.index.name = "Segment(i)"

st.write("Insert Segment Data")
st.session_state["robot_data"] = st.data_editor(robot_df)

if st.button("Run Simulation"):
    u.export_robot_json(st.session_state["robot_data"])