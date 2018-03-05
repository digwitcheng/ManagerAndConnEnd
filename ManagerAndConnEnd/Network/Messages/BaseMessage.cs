using AGV_V1._0.Event;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    public abstract class BaseMessage
    {
        protected const int SUCCESS = 6;
        protected const int FAIRED = -1;
        public MessageType Type { get;  set; }
        public string Message { get; set; }
        public abstract BaseMessage Create(string msg);
        public abstract void Send(string sessionKey, TcpSocketServer _server);
        public abstract void Receive();

        public event EventHandler<MessageEventArgs> ShowMessage;
        public event EventHandler<MessageEventArgs> DataMessage;
        public delegate void ReLoadDele();
        public ReLoadDele ReLoad;

        public static BaseMessage Factory(MessageType type, string rawMessage)
        {
            switch (type)
            {
                case MessageType.DisConnect:
                    return new DisConnMessage().Create( rawMessage);                
                case MessageType.Arrived:
                    return new ArrivedMessage().Create(rawMessage);
                case MessageType.ReStart:
                    return new ReStartMessage().Create(rawMessage);
                case MessageType.Move:
                    return new MoveMessage().Create(rawMessage);
                case MessageType.Msg:
                    return new MsgMessage().Create(rawMessage);
                case MessageType.AgvFile:
                    return new AgvFileMessage().Create(rawMessage);
                case MessageType.MapFile:
                    return new MapFileMessage().Create(rawMessage);
                default:
                    return new ErrorMessage().Create(rawMessage);
            }

        }

        protected void OnMessageEvent(string message)
        {
            try
            {
                if (null != ShowMessage)
                {
                    ShowMessage.Invoke(this, new MessageEventArgs(message));
                }
            }
            catch
            {

            }
        }
        protected void OnTransmitEvent(object sender, MessageEventArgs e)
        {
            try
            {
                if (null != DataMessage)
                {
                    DataMessage.Invoke(sender, e);
                }
            }
            catch
            {

            }
        }
        
    }
}
