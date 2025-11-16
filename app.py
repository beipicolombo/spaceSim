import numpy as np
import time

import streamlit as st
import plotly.graph_objects as go
import pandas as pd

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu

st.title("SpaceSim")

# Run scenario button
# choice = st.selectbox("Select scenario", list(AVAILABLE_SCENARIOS.keys()))
choice = "testDevelopment" # [TBW] Unique choice for now
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS[choice]

runButton = st.button("Run simulation")

# Setup layout
col1, col2 = st.columns(2)

tileModeMgt = col1.container(border = True)
tileModeMgt.title("FSW")

tilePlots = col2.container(border = True)
tilePlots.title("Models")

idxFig = 0
allFigDict = {}

def addFigure(allFigDict, name):
    newFig = go.Figure()
    allFigDict.update({name: newFig})
    return (newFig, allFigDict)

if runButton:

    # Run simulation
    with st.spinner("Running...", show_time=True):
        # time.sleep(5)
        out = runSimu(scenarioPatchFcnHdl)
    st.success("Done!")

    # Get signals to display
    time = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.toMin().timeVec
    angles = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().dataVec
    angRate = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().dataVec
    angRateNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.getNorm().dataVec
    scAngMom = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.dataVec
    scAngMomNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.getNorm().dataVec
    isAocsModeTrans = out["fswBus"].subBuses["modeMgt"].signals["isAocsModeTrans"].timeseries.dataVec.squeeze()
    aocsMode = out["fswBus"].subBuses["modeMgt"].signals["aocsMode"].timeseries.dataVec.squeeze()
    # Get data to display
    allEvtDataframe = out["allEvtDataframe"]


    # Mode management
    # ==================================
    with tileModeMgt:
        # Write events summary
        st.write("EVENTS")
        st.dataframe(allEvtDataframe)

        # AOCS modes summary [TBW]
        (currFig, allFigDict) = addFigure(allFigDict, "aocsMode")
        currFig.add_trace(go.Scatter(x=time, y=aocsMode))
        currFig.update_layout(title="AOCS Mode",
                           xaxis_title = "Time [min]",
                           yaxis_title = "-")
        st.plotly_chart(currFig, use_container_width=True)

        (currFig, allFigDict) = addFigure(allFigDict, "aocsModeTrans")
        currFig.add_trace(go.Scatter(x=time, y=isAocsModeTrans))
        currFig.update_layout(title="AOCS mode transition flag",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[-]")
        st.plotly_chart(currFig, use_container_width=True)

    # Plots
    # ==================================
    with tilePlots:
        # Angular rates plot
        (currFig, allFigDict) = addFigure(allFigDict, "angRates")
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,0], name="x"))
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,1], name="y"))
        currFig.add_trace(go.Scatter(x=time, y=angRate[:,2], name="z"))
        currFig.add_trace(go.Scatter(x=time, y=angRateNorm, name="norm"))
        currFig.update_layout(title="Angular rates",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg/s]")
        st.plotly_chart(currFig, use_container_width=True)

        # Angular momentum plot
        (currFig, allFigDict) = addFigure(allFigDict, "angMom")
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,0], name="x"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,1], name="y"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMom[:,2], name="z"))
        currFig.add_trace(go.Scatter(x=time, y=scAngMomNorm, name="norm"))
        currFig.update_layout(title="Angular momentum",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[Nms]")
        st.plotly_chart(currFig, use_container_width=True)

        # Euler angles attitude plot
        (currFig, allFigDict) = addFigure(allFigDict, "attitudeEulerAng")
        currFig.add_trace(go.Scatter(x=time, y=angles[:,0], name="roll"))
        currFig.add_trace(go.Scatter(x=time, y=angles[:,1], name="pitch"))
        currFig.add_trace(go.Scatter(x=time, y=angles[:,2], name="yaw"))
        currFig.update_layout(title="Attitude",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg]")
        st.plotly_chart(currFig, use_container_width=True)
