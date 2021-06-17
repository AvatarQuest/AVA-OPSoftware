//======= Copyright (c) Valve Corporation, All rights reserved. ===============
//
// Purpose:	CameraFade script adapted to work with SteamVR.
//
// Usage:	Add to your top level SteamVR_Camera (the one with ApplyDistoration
//			checked) and drag a reference to this component into SteamVR_Camera
//			RenderComponents list.  Then call the static helper function
//			SteamVR_Fade.Start with the desired color and duration.
//			Use a duration of zero to set the start color.
//
// Example:	Fade down from black over one second.
//			SteamVR_Fade.Start(Color.black, 0);
//			SteamVR_Fade.Start(Color.clear, 1);
//
// Note:	This component is provided to fade out a single camera layer's
//			scene view.  If instead you want to fade the entire view, use:
//			SteamVR_Fade.View(Color.black, 1);
//			(Does not affect the game view, however.)
//
//=============================================================================

using UnityEngine;
using Valve.VR;
using UnityEngine.VR;
using System;

using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROSBridgeLib;
using SimpleJSON;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.std_msgs;

namespace Valve.VR
{   
    public class logRotation : MonoBehaviour
    {   
        Rigidbody rb;
        GameObject rosObj;
        Vector3Msg msg;
    
  

        private Color currentColor = new Color(0, 0, 0, 0); // default starting color: black and fully transparent
        private Color targetColor = new Color(0, 0, 0, 0);  // default target color: black and fully transparent
        private Color deltaColor = new Color(0, 0, 0, 0);   // the delta-color is basically the "speed / second" at which the current color should change
        private bool fadeOverlay = false;

        static public void Start(Color newColor, float duration, bool fadeOverlay = false)
        {
            SteamVR_Events.Fade.Send(newColor, duration, fadeOverlay);
        }

        static public void View(Color newColor, float duration)
        {
            var compositor = OpenVR.Compositor;
            if (compositor != null)
                compositor.FadeToColor(duration, newColor.r, newColor.g, newColor.b, newColor.a, false);
        }

#if TEST_FADE_VIEW
	void Update()
	{
		if (Input.GetKeyDown(KeyCode.Space))
		{
			SteamVR_Fade.View(Color.black, 0);
			SteamVR_Fade.View(Color.clear, 1);
		}

        
	}
#endif


        public void OnStartFade(Color newColor, float duration, bool fadeOverlay)
        {
            if (duration > 0.0f)
            {
                targetColor = newColor;
                deltaColor = (targetColor - currentColor) / duration;
            }
            else
            {
                currentColor = newColor;
            }
        }

        static Material fadeMaterial = null;
        static int fadeMaterialColorID = -1;

        void OnEnable()
        {
            if (fadeMaterial == null)
            {
                fadeMaterial = new Material(Shader.Find("Custom/SteamVR_Fade"));
                fadeMaterialColorID = Shader.PropertyToID("fadeColor");
            }

            SteamVR_Events.Fade.Listen(OnStartFade);
            SteamVR_Events.FadeReady.Send();
        }

        void OnDisable()
        {
            SteamVR_Events.Fade.Remove(OnStartFade);
        }

        void OnPostRender()
        {
            if (currentColor != targetColor)
            {
                // if the difference between the current alpha and the desired alpha is smaller than delta-alpha * deltaTime, then we're pretty much done fading:
                if (Mathf.Abs(currentColor.a - targetColor.a) < Mathf.Abs(deltaColor.a) * Time.deltaTime)
                {
                    currentColor = targetColor;
                    deltaColor = new Color(0, 0, 0, 0);
                }
                else
                {
                    currentColor += deltaColor * Time.deltaTime;
                }

                if (fadeOverlay)
                {
                    var overlay = SteamVR_Overlay.instance;
                    if (overlay != null)
                    {
                        overlay.alpha = 1.0f - currentColor.a;
                    }
                }
            }

            if (currentColor.a > 0 && fadeMaterial)
            {
                fadeMaterial.SetColor(fadeMaterialColorID, currentColor);
                fadeMaterial.SetPass(0);
                GL.Begin(GL.QUADS);

                GL.Vertex3(-1, -1, 0);
                GL.Vertex3(1, -1, 0);
                GL.Vertex3(1, 1, 0);
                GL.Vertex3(-1, 1, 0);
                GL.End();
            }
        }

        // Xtra code to remove if it doesn't work! - Rishi
        private const string logPrefix = "InputTrackerChecker: ";

        [SerializeField] UnityEngine.XR.XRNode m_VRNode = UnityEngine.XR.XRNode.Head;

        private void Start() {
            //Since we attached ROSInitializer to Main Camera:
            rosObj = GameObject.Find("ROSInitializer");
            //rosObj = camera.Main;
            rb = GetComponent<Rigidbody>();
            StartCoroutine(EndOfFrameUpdate());
        }

        private void Update() {
            LogRotation("Update");
        }

        private void LateUpdate() {
            LogRotation("LateUpdate");
        }

        private IEnumerator EndOfFrameUpdate() {
            while (true) {
                yield return new WaitForEndOfFrame();
                LogRotation("EndOfFrame");
            }
        }

        private void LogRotation( string id ) {
            var quaternion = UnityEngine.XR.InputTracking.GetLocalRotation(m_VRNode);
            var euler = quaternion.eulerAngles;

            var x = euler.x; //euler.GetX
            var y = euler.y; //euler.GetY
            var z = euler.z; //euler.GetZ

            msg = new Vector3Msg(x,y,z);
            Debug.Log("Sending MSG");
            rosObj.GetComponent<ROSInitializer>().ros.Publish(
                PanTiltPublisher.GetMessageTopic(), msg
            );

            // // print("{0} {1}, ({2}) Quaternion {3} Euler {4}", logPrefix, id, m_VRNode, quaternion.ToString("F2"), euler.ToString("F2"));
            // Debug.Log(string.Format("{0} {1}, ({2}) Quaternion {3} Euler {4}", logPrefix, id, m_VRNode, quaternion.ToString("F2"), euler.ToString("F2")));

            // // And in some other class where the ball is controlled:
            // TwistMsg msg = new TwistMsg(x, y, z); // Circa

            // // Publish it (ros is the object defined in the first class)
            // ros.Publish(BallControlPublisher.GetMessageTopic(), msg);

            // bool hasCompleted = writeFile(string.Format(quaternion.ToString("F2")));

            // if (hasCompleted) {
            //     Debug.Log(string.Format("Done!"));
            // } else {
            //     Debug.Log(string.Format("Error occurred!"));
            // }
        }   

        private bool writeFile(string toType) {
            string folder = @"C:\Users\avatar\Desktop\avatarCam\";
            string textFileName = "TesterFile.txt";

            string fullPathOfTextFile = folder + textFileName;

            toType += "\n";

            File.WriteAllText(fullPathOfTextFile, toType);    

            string[] stringsReturned = readFile();

            return true;
        }

        private string[] readFile() {
            string folder = @"C:\Users\avatar\Desktop\avatarCam\";
            string textFileName = "TesterFile.txt";

            string fullPathOfTextFile = folder + textFileName;

            return File.ReadAllLines(fullPathOfTextFile);
        }
    }
}
