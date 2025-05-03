
import matplotlib.pyplot as plt


def runPlots(modelsBus, fswBus, simParam):

    if simParam.runOptions.swPlot:
        figuresList = []

        #plt.show(block = False)
        # plots.plotOrbit3d(modelsBus.subBuses["dynamics"].subBuses["posVel"].signals["pos_I"].timeseries, const.earthRadius)
        
        figuresList.append(modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angRate_BI_B"].timeseries.rad2deg().plot())
        # fswBus.subBuses["guidance"].signals["angRate_RI_R"].timeseries.rad2deg().plot()
        
        figuresList.append(modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["eulerAng_BI"].timeseries.rad2deg().plot())
        # fswBus.subBuses["guidance"].signals["eulerAng_RI"].timeseries.rad2deg().plot()
        # modelsBus.subBuses["environment"].signals["eulerAng_LI"].timeseries.rad2deg().plot()
        # modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["torqueTot_B"].timeseries.addNorm().plot()
        # #modelsBus.subBuses["dynamics"].subBuses["attitude"].signals["angMomSc_B"].timeseries.addNorm().plot()
        
        # fswBus.subBuses["control"].signals["eulerAngEst_BR"].timeseries.rad2deg().plot()
        # modelsBus.subBuses["performance"].signals["eulerAng_BL"].timeseries.rad2deg().plot()
        # modelsBus.subBuses["performance"].signals["angRate_BL_B"].timeseries.rad2deg().plot()
        
        # # modelsBus.subBuses["sensors"].signals["angRateMeas_BI_B"].timeseries.addNorm().rad2deg().plot()
        figuresList.append(modelsBus.subBuses["actuators"].signals["torqueThr_B"].timeseries.addNorm().plot())
        
        # #modelsBus.subBuses["environment"].signals["torqueExt_B"].timeseries.addNorm().plot()
        
        # # fswBus.subBuses["guidance"].signals["angRate_RI_R"].timeseries.rad2deg().plot()
        # # fswBus.subBuses["guidance"].signals["eulerAng_RI"].timeseries.rad2deg().plot()
        
        # fswBus.subBuses["control"].signals["qEstBR_sca"].timeseries.plot()
        # fswBus.subBuses["control"].signals["qEstBR_vec"].timeseries.plot()
        # fswBus.subBuses["control"].signals["rotAngEst_BR"].timeseries.rad2deg().plot()
        # fswBus.subBuses["control"].signals["angRateEst_BR_B"].timeseries.rad2deg().plot()
        # #fswBus.subBuses["control"].signals["forceCtrl_B"].timeseries.addNorm().plot()
        # #fswBus.subBuses["control"].signals["torqueCtrl_B"].timeseries.addNorm().plot()
        # fswBus.subBuses["control"].signals["torqueCtrlThr_B"].timeseries.plot()
        # #fswBus.subBuses["control"].signals["torqueCtrlRw_B"].timeseries.plot()
        
        # #fswBus.subBuses["command"].signals["torqueCmdRw_B"].timeseries.addNorm().plot()
        # #fswBus.subBuses["command"].signals["torqueCmdThr_B"].timeseries.addNorm().plot()
        
        # #fswBus.subBuses["modeMgt"].signals["aocsModeElapsedTime"].timeseries.plot()
        
        # modelsBus.subBuses["environment"].signals["qLI_sca"].timeseries.plot()
        # modelsBus.subBuses["environment"].signals["qLI_vec"].timeseries.plot()
        
        # modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["ta"].timeseries.rad2deg().plot()
        # modelsBus.subBuses["dynamics"].subBuses["orbitElem"].signals["argPer"].timeseries.rad2deg().plot()

        for fig in figuresList:
            plt.show(block = False)