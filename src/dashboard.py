import streamlit as st
import pandas as pd
import utils as u
import dashboard_helper_funcs as f

materials = u.import_json_data("materials")

st.set_page_config(layout="wide")
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
            "n_waypoints": None,
            "waypoints": None,
            },
        "results": None
    }

with st.sidebar:
    if st.button("Clear Memory"):
        u.clear_imgs_folder()
        st.session_state["data"] = None
        st.rerun()

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
        st.session_state["data"]["n_waypoints"] = st.number_input(label="Number of waypoints", min_value=1)
        st.markdown("##### Specify X Targets")
        waypoints = pd.DataFrame(
            {"x": [0.0] * st.session_state["data"]["n_waypoints"]}, index=[i for i in range(1, st.session_state["data"]["n_waypoints"]+1)])
        waypoints.index.name="Waypoint(w)"  
        st.session_state["data"]["waypoints"] = st.data_editor(waypoints, width="content")

col1, col2 = st.columns(2)

with col1:
    st.markdown("#### Insert Segment Data")
    st.session_state["data"]["robot_data"] = f.create_segment_df(st.session_state["data"]["n_segments"])

    if st.button("Run Simulation"):
        st.session_state["data"]["robot_data"] = st.session_state["data"]["robot_data"].to_dict()
        if st.session_state["data"]["inverse_kinematics"]:
            st.session_state["data"]["waypoints"] = st.session_state["data"]["waypoints"].to_dict()
        u.export_json(st.session_state["data"], "data")
        st.session_state["results"] = f.run_analysis()

if st.session_state["results"] is not None:
    with col2:
        f.display_initial_pose(st.session_state["results"])
    st.markdown("**Final Statistics**")
    st.dataframe(st.session_state["results"]["df"])