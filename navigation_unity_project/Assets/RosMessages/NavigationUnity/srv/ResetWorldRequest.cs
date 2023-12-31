//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.NavigationUnity
{
    [Serializable]
    public class ResetWorldRequest : Message
    {
        public const string k_RosMessageName = "navigation_unity_msgs/ResetWorld";
        public override string RosMessageName => k_RosMessageName;

        public string world_config;

        public ResetWorldRequest()
        {
            this.world_config = "";
        }

        public ResetWorldRequest(string world_config)
        {
            this.world_config = world_config;
        }

        public static ResetWorldRequest Deserialize(MessageDeserializer deserializer) => new ResetWorldRequest(deserializer);

        private ResetWorldRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.world_config);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.world_config);
        }

        public override string ToString()
        {
            return "ResetWorldRequest: " +
            "\nworld_config: " + world_config.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
