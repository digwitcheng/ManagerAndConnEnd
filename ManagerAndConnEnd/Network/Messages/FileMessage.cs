using AGV_V1._0.Event;
using AGV_V1._0.NLog;
using AGV_V1._0.Util;
using Cowboy.Sockets;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.Messages
{
    class FileMessage:BaseMessage
    {
        public override BaseMessage Create(string msg)
        {
            return new FileMessage();
        }

        public override  void Send(string sessionKey, Cowboy.Sockets.TcpSocketServer _server)
        {
            SendFile(sessionKey, _server);
        }

        public override  void Receive()
        {
            try
            {
                OnMessageEvent("接收中...");
                string pathAgv = ConstDefine.AGV_PATH;
                //Task.Factory.StartNew(() => ReciveFile(pathAgv, e), TaskCreationOptions.LongRunning);
                ReciveFile(pathAgv, this.Message);
                OnMessageEvent("文件接收成功");
                OnMessageEvent("文件转发中...");
                OnTransmitEvent(this, new MessageEventArgs(MessageType.AgvFile, pathAgv));
                OnMessageEvent("文件转发成功");
            }
            catch (Exception ex)
            {
                Logs.Error("文件接收失败" + ex);
                OnMessageEvent("文件接收失败" + ex);
            }
        }

        void ReciveFile(string path, string msg)
        {
            try
            {
                using (FileStream fswrite = new FileStream(path, FileMode.Create, FileAccess.Write))
                {
                    //fswrite.Write(e.Data, e.DataOffset + 1, e.DataLength - 1);
                    StreamWriter sw = new StreamWriter(fswrite);
                    sw.Write(msg);
                    sw.Flush();
                }
                ReLoad();

            }
            catch (Exception ex)
            {
                Logs.Error("文件接收失败" + ex);
                throw ;
            }
        }

        /// <summary>
        /// 发送文件
        /// </summary>
        /// <param name="path">文件路径</param>
        /// <param name="flag">文件类型</param>
        int SendFile(string sessionKey, TcpSocketServer _server)
        {
            if (!File.Exists(this.Message))
            {
                OnMessageEvent("文件不存在");
                Logs.Error("文件不存在");
                return FAIRED;
            }
            try
            {
                using (FileStream fsRead = new FileStream(this.Message, FileMode.OpenOrCreate, FileAccess.Read))
                {
                    byte[] buffer = new byte[1024 * 1024 * 3];
                    int r = fsRead.Read(buffer, 0, buffer.Length);
                    List<byte> list = new List<byte>();
                    list.Add((byte)this.Type);
                    list.AddRange(buffer);
                    byte[] newBuffer = list.ToArray();
                    list.Clear();

                    if (sessionKey.Trim() == "")
                    {
                        _server.Broadcast(newBuffer, 0, r + 1);
                    }
                    else
                    {
                        _server.SendTo(sessionKey, newBuffer, 0, r + 1);
                    }
                    // MessageBox.Show("文件已发送~~");
                    OnMessageEvent("文件发送成功");

                }
                return SUCCESS;
            }
            catch (Exception ex)
            {
                Logs.Error("文件发送失败" + ex);
                OnMessageEvent("文件发送失败" + ex);
                throw new FileNotFoundException();
            }
        }
    }
}
