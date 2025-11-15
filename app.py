import streamlit as st
import numpy as np
import plotly.graph_objects as go

import src.data.scenarios.scenarioDefinitions as scenarioDefinitions
from src.data.scenarios.scenarioDefinitions import AVAILABLE_SCENARIOS
from src.utils.runSimu import runSimu
import pandas as pd

st.title("SpaceSim")

choice = st.selectbox("Select scenario", list(AVAILABLE_SCENARIOS.keys()))
scenarioPatchFcnHdl = AVAILABLE_SCENARIOS[choice]
# result = fn(10)
# st.write(result)

if st.button("Run simulation"):

    # Run simulation
    out = runSimu(scenarioPatchFcnHdl)

    # Get signals to display
    time = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.toMin().timeVec
    angles = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().dataVec
    angRate = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().dataVec
    angRateNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.getNorm().dataVec
    scAngMom = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.dataVec
    scAngMomNorm = out["modelsBus"].subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.getNorm().dataVec
    isAocsModeTrans = out["fswBus"].subBuses["modeMgt"].signals["isAocsModeTrans"].timeseries.dataVec.squeeze()
    # Get data to display
    allEvtDataframe = out["allEvtDataframe"]

    # Initialize columns
    col1, col2 = st.columns(2)

    # Events
    # ==================================
    # Write events summary
    with col1:
        st.write("EVENTS")
        st.dataframe(allEvtDataframe)

    # Temporal data
    # ==================================
    with col2:
        # Initialize figures layout
        fig1 = go.Figure()
        fig2 = go.Figure()
        fig3 = go.Figure()
        fig4 = go.Figure()
    
        # Update figures
        fig1.add_trace(go.Scatter(x=time, y=angRate[:,0], name="x"))
        fig1.add_trace(go.Scatter(x=time, y=angRate[:,1], name="y"))
        fig1.add_trace(go.Scatter(x=time, y=angRate[:,2], name="z"))
        fig1.add_trace(go.Scatter(x=time, y=angRateNorm, name="norm"))
        fig1.update_layout(title="Angular rates",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg/s]")
        st.plotly_chart(fig1, use_container_width=True)

        fig2.add_trace(go.Scatter(x=time, y=scAngMom[:,0], name="x"))
        fig2.add_trace(go.Scatter(x=time, y=scAngMom[:,1], name="y"))
        fig2.add_trace(go.Scatter(x=time, y=scAngMom[:,2], name="z"))
        fig2.add_trace(go.Scatter(x=time, y=scAngMomNorm, name="norm"))
        fig2.update_layout(title="Angular momentum",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[Nms]")
        st.plotly_chart(fig2, use_container_width=True)

        fig3.add_trace(go.Scatter(x=time, y=angles[:,0], name="roll"))
        fig3.add_trace(go.Scatter(x=time, y=angles[:,1], name="pitch"))
        fig3.add_trace(go.Scatter(x=time, y=angles[:,2], name="yaw"))
        fig3.update_layout(title="Attitude",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[deg]")
        st.plotly_chart(fig3, use_container_width=True)

        fig4.add_trace(go.Scatter(x=time, y=isAocsModeTrans, name="isAocsModeTrans"))
        fig4.update_layout(title="isAocsModeTrans",
                           xaxis_title = "Time [min]",
                           yaxis_title = "[-]")
        st.plotly_chart(fig4, use_container_width=True)

    # st.write("Mode timeline:")
    # st.write(mode[:20])  # or plot as categorical
