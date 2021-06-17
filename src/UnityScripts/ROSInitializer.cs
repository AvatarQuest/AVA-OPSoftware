using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROSBridgeLib;
using ROSBridgeLib.std_msgs;

public class ROSInitializer : MonoBehaviour
{

  public Color color = Color.black;

   public ROSBridgeWebSocketConnection ros = null;
    
  void Start() {
    // Where the rosbridge instance is running, could be localhost, or some external IP
    ros = new ROSBridgeWebSocketConnection ("ws://10.59.0.70", 9090);

    // Add subscribers and publishers (if any)
    // ros.AddSubscriber(typeof(UnitySubscriber));
    ros.AddPublisher(typeof(PanTiltPublisher));
    ros.AddPublisher(typeof(ArmPositionPublisher));

    // Fire up the subscriber(s) and publisher(s)
    ros.Connect ();

    Camera.main.backgroundColor = color;
    
  }
  
  // Extremely important to disconnect from ROS. Otherwise packets continue to flow
  void OnApplicationQuit() {
    if(ros!=null) {
      ros.Disconnect ();
    }
  }
  // Update is called once per frame in Unity
  void Update () {
    ros.Render ();
    // StringMsg msg = new StringMsg("hi pls respond");
    // ros.Publish(UnityPublisher.GetMessageTopic(), msg);
    // Debug.Log("Sent message");
  }
}
