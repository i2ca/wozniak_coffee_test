using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class StringSubscriber : UnitySubscriber<MessageTypes.Std.String>
    {
        //public Transform PublishedTransform;

        public string receivedString;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Std.String message)
        {
            receivedString = GetStrings(message);
            isMessageReceived = true;
        }
        private void ProcessMessage()
        {
            //PublishedTransform.rotation = rotation;
        }


        private string GetStrings(MessageTypes.Std.String message)
        {
            return message.data;
        }
    }
}
