import numpy as np
import time

import streamlit as st
import plotly.graph_objects as go
import pandas as pd

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu

st.set_page_config(page_title = "AOCS Mission Control", layout = "wide")



# ============================================================================
# 1) Initialize session_state for data and readiness
# ============================================================================

if "data_ready" not in st.session_state:
    st.session_state.data_ready = False
    st.session_state.time = None
    st.session_state.angles = None
    st.session_state.angRate = None
    st.session_state.angRateNorm = None
    st.session_state.scAngMom = None
    st.session_state.scAngMomNorm = None
    st.session_state.isAocsModeTrans = None
    st.session_state.aocsMode = None
    st.session_state.allEvtDataframe = None

# ============================================================================
# 2) Build the dashboard layout FIRST
# ============================================================================

# -----------------------------------------------------
# Title Section
# -----------------------------------------------------

st.title("SpaceSim Dashboard")
st.caption("Mission Control and Flight Simulator")

# Run scenario button
# choice = st.selectbox("Select scenario", list(AVAILABLE_SCENARIOS.keys()))
choice = "testDevelopment" # [TBW] Unique choice for now
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS[choice]

runButton = st.button("Launch simulation")


# -----------------------------------------------------
# Mission Launch Section
# -----------------------------------------------------
# [TBW] for real time environment


# -----------------------------------------------------
# Mission Summary Section
# -----------------------------------------------------
col1, col2, col3, col4 = st.columns(4)

# [TBW] Implement placeholders or refresh
col1.metric("AOCS Mode [Dum]", "Nadir Pointing")
col2.metric("Body Rates (RMS) [Dum]", f"{10:.4f} deg/s")
col3.metric("Pointing Error [Dum]", "10 deg")
col4.metric("Thruster Events [Dum]", f"{10}")

st.divider()


# -----------------------------------------------------
# [TBW] Telemetry Section
# -----------------------------------------------------

# Define tabs
tabDyn, tabAocsModes, tabEqpt = st.tabs([
    "Spacecraft Dynamics",
    "AOCS Modes",
    "Equipment"
])

# Create placeholders inside tabs
with tabDyn:
    angRates_placeholder = st.empty()
    angMom_placeholder = st.empty()
    attitudeEulerAng_placeholder = st.empty()

with tabAocsModes:
    evtDataFrame_placeholder = st.empty()
    aocsMode_placeholder = st.empty()
    isAocsModeTrans_placeholder = st.empty()


# ============================================================================
# 3) Launch simulation and retrieve outputs when buttong is pressed
# ============================================================================
if runButton:
    with st.spinner("Running...", show_time=True):
        # time.sleep(5)
        out = runSimu(scenarioPatchFcnHdl)
    st.success("Done! Dashboard updated.")

    # Set the data states
    st.session_state.time = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.toMin().timeVec
    st.session_state.angles = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().dataVec
    st.session_state.angRate = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().dataVec
    st.session_state.angRateNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.getNorm().dataVec
    st.session_state.scAngMom = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.dataVec
    st.session_state.scAngMomNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.getNorm().dataVec
    st.session_state.isAocsModeTrans = out["fswBus"].subBuses["modeMgt"].signals["isAocsModeTrans"].timeseries.dataVec.squeeze()
    st.session_state.aocsMode = out["fswBus"].subBuses["modeMgt"].signals["aocsMode"].timeseries.dataVec.squeeze()
    st.session_state.allEvtDataframe = out["allEvtDataframe"]
    # Set the data states to ready
    st.session_state.data_ready = True


# ============================================================================
# 4) Update the dashboard when data state is ready
# ============================================================================

def addFigure(allFigDict, name):
    newFig = go.Figure()
    allFigDict.update({name: newFig})
    return (newFig, allFigDict)


if st.session_state.data_ready:

    # Retrieve data
    time = st.session_state.time
    angles = st.session_state.angles
    angRate = st.session_state.angRate
    angRateNorm = st.session_state.angRateNorm
    scAngMom = st.session_state.scAngMom
    scAngMomNorm = st.session_state.scAngMomNorm
    isAocsModeTrans = st.session_state.isAocsModeTrans
    aocsMode = st.session_state.aocsMode
    allEvtDataframe = st.session_state.allEvtDataframe

    # Define figures and update placeholders
    allFigDict = {}

    # Spacecraft dynamics
    with tabDyn:
        
        # st.subheader("Spacecraft Dynamics")

        # Angular rates plot
        (currFig, allFigDict) = addFigure(allFigDict, "angRates")
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,0], name="x"))
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,1], name="y"))
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,2], name="z"))
        currFig.add_trace(go.Scatter(x=time, y=angRateNorm, name="norm"))
        currFig.update_layout(title="Angular rates",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg/s]")
        # Update placeholder
        angRates_placeholder.plotly_chart(currFig, use_container_width=True)

        # Angular momentum plot
        (currFig, allFigDict) = addFigure(allFigDict, "angMom")
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,0], name="x"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,1], name="y"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,2], name="z"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMomNorm, name="norm"))
        currFig.update_layout(title="Angular momentum",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[Nms]")
        # Update placeholder
        angMom_placeholder.plotly_chart(currFig, use_container_width=True)

        # Euler angles attitude plot
        (currFig, allFigDict) = addFigure(allFigDict, "attitudeEulerAng")
        currFig.add_trace(go.Scatter(x=time, y=angles[:,0], name="roll"))
        currFig.add_trace(go.Scatter(x=time, y=angles[:,1], name="pitch"))
        currFig.add_trace(go.Scatter(x=time, y=angles[:,2], name="yaw"))
        currFig.update_layout(title="Attitude",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg]")
        # Update placeholder
        attitudeEulerAng_placeholder.plotly_chart(currFig, use_container_width=True)      


    # AOCS modes
    with tabAocsModes:

        # st.subheader("AOCS Modes")

        # Write events summary
        evtDataFrame_placeholder.write("EVENTS")
        evtDataFrame_placeholder.dataframe(allEvtDataframe)

        # AOCS modes summary [TBW]
        (currFig, allFigDict) = addFigure(allFigDict, "aocsMode")
        currFig.add_trace(go.Scatter(x=time, y=aocsMode))
        currFig.update_layout(title="AOCS Mode",
                           xaxis_title = "Time [min]",
                           yaxis_title = "-")
        aocsMode_placeholder.plotly_chart(currFig, use_container_width=True)

        (currFig, allFigDict) = addFigure(allFigDict, "aocsModeTrans")
        currFig.add_trace(go.Scatter(x=time, y=isAocsModeTrans))
        currFig.update_layout(title="AOCS mode transition flag",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[-]")
        isAocsModeTrans_placeholder.plotly_chart(currFig, use_container_width=True)
