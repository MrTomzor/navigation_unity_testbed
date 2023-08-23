using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Unity.Robotics.Core;

public class SimUIManager : MonoBehaviour
{
    public bool manualControlOn = true;
    public bool debugCameraOn = false;

    public GameObject controlledRobot;
    public GameObject debugCamera;
    public GameObject mainStatsBackground;
    public TMP_Text mainStatsText;
    
    double rtfMeasurePeriod = 1;
    double rtfNextMeasure = 1;
    double rtfLastClocktime;

    double measuredRTF = 0;
    RenderTexture texture;


    // Start is called before the first frame update
    void Start()
    {

      rtfNextMeasure = Time.realtimeSinceStartup + rtfMeasurePeriod;
      rtfLastClocktime = Clock.time;
      texture = new RenderTexture (800, 480, 24);

      debugCameraOn = false;
      debugCamera.SetActive(false);
    }

    void RefreshText(){
      mainStatsText.text = "";
      mainStatsText.text += "ManualControl: ";
      mainStatsText.text += manualControlOn ? "ON" : "OFF (press M)";
      mainStatsText.text += "\n";

      mainStatsText.text += "DebugCamera: ";
      mainStatsText.text += debugCameraOn ? "ON" : "OFF (press R)";
      mainStatsText.text += "\n";

      mainStatsText.text += "RealTime: " + Time.realtimeSinceStartup;
      mainStatsText.text += "\n";
      mainStatsText.text += "RosTime: " + Clock.time;
      mainStatsText.text += "\n";
      mainStatsText.text += "RTF: " + measuredRTF;
      mainStatsText.text += "\n";
    }

    // Update is called once per frame
    void Update()
    {
      if(Input.GetKeyDown("r")){
        debugCameraOn = !debugCameraOn;
        debugCamera.SetActive( debugCameraOn);
        mainStatsBackground.SetActive(!debugCameraOn);
      }

      if(Input.GetKeyDown("m")){
        manualControlOn = !manualControlOn;
        controlledRobot.SendMessage("ToggleManualControl", manualControlOn);
      }

      double realtime = Time.realtimeSinceStartup;

      /* Debug.Log("realtime: " + realtime + "next measure:" + rtfNextMeasure); */
      if(realtime > rtfNextMeasure){
        double rtfNowClocktime = Clock.time;
        double realtimeNowTime = realtime;

        double dtimeROS = rtfNowClocktime - rtfLastClocktime;
        double dtimeWall = realtime - (rtfNextMeasure - rtfMeasurePeriod);

        measuredRTF = dtimeROS / dtimeWall;

        rtfLastClocktime = rtfNowClocktime;
        rtfNextMeasure = rtfNextMeasure + rtfMeasurePeriod;

      }
        
      GL.Clear(true, true, Color.black);

      /* Camera.main.targetTexture = null; */
      /* Graphics.DrawTexture (new Rect (0, 0, Screen.width, Screen.height), texture); */

      RefreshText();
    }
}
