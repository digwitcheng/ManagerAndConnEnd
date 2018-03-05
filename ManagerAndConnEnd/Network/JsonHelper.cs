using AGV_V1._0.Agv;
using AGV_V1._0.Algorithm;
using Agv.PathPlanning;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0
{
    class JsonHelper
    {
        public static string VehicleToJson(Vehicle v)
        {
            SendData obj = GetSendObj(v);
            return Newtonsoft.Json.JsonConvert.SerializeObject(obj);
        }
        public static string VehiclesToJson(List<Vehicle> v)
        {
            SendData[] sendObj = new SendData[v.Count()];
            for (int i = 0; i < v.Count(); i++)
            {
                SendData obj = GetSendObj(v[i]);
                sendObj[i] = obj;
            }

            return Newtonsoft.Json.JsonConvert.SerializeObject(sendObj);
        }
        public static string VehiclesToGuiJson(Vehicle[] v)
        {
            
            GuiData[] sendObj = new GuiData[v.Count()];
            for (int i = 0; i < v.Count(); i++)
            {
                GuiData obj = GetGuiObj(v[i]);
                sendObj[i] = obj;
            }
            string str = Newtonsoft.Json.JsonConvert.SerializeObject(sendObj);
            

            return str;
        }
        private static GuiData GetGuiObj(Vehicle v)
        {
            lock (RouteLock)
            {
                if (v == null)
                {
                    return null;
                }
                GuiData obj = new GuiData();
                obj.Num = (ushort)v.Id;
                //int tPtr = v.TPtr;
                obj.BeginX = (byte)v.BeginX;
                obj.BeginY = (byte)v.BeginY;
                obj.State = (byte)v.CurState;
                return obj;
            }
        }
        public static string VehiclesToJson(Vehicle[] v)
        {
            SendData[] sendObj = new SendData[v.Count()];
            for (int i = 0; i < v.Count(); i++)
            {
                SendData obj = GetSendObj(v[i]);
                sendObj[i] = obj;
            }

            return Newtonsoft.Json.JsonConvert.SerializeObject(sendObj);
        }
        //加一个锁
        private static Object RouteLock = new Object();
        private static SendData GetSendObj(Vehicle v)
        {
            lock (RouteLock)
            {
                if (v == null)
                {
                    return null;
                }
                SendData obj = new SendData();
                obj.Num = v.Id;
                //int tPtr = v.TPtr;
                obj.BeginX = v.BeginX;
                obj.BeginY = v.BeginY;
                //List<MyLocation> route = v.GetRoute();
                //if (route != null)
                //{
                //    if (route.Count > 0)
                //    {
                //        if (tPtr < route.Count)
                //        {
                //            obj.BeginX = route[tPtr].Point.X;
                //            obj.BeginY = route[tPtr].Point.Y;
                //        }
                //        else
                //        {
                //            obj.BeginX = route[route.Count - 1].Point.X;
                //            obj.BeginY = route[route.Count - 1].Point.Y;
                //        }
                //    }
                //}
            
            obj.State = v.CurState;
            obj.Arrive = v.Arrive;
            obj.Direction = v.Dir;
            obj.EndLoc = v.EndLoc;            
            if (v.CurState == State.cannotToDestination)
            {
                obj.StartLoc = v.StartLoc;
            }
            else
            {
                obj.StartLoc = v.EndLoc;
            }
            return obj;
          }
        }
        /// <summary>
        /// T代表要返回的数据类型
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="json"></param>
        /// <returns></returns>
        public static List<T> JsonToData<T>(string json)
        {
            lock (RouteLock)
            {
                if (json != null)
                {
                    List<T> listData = new List<T>();
                    int start = json.IndexOf("{", 0);
                    int end = json.IndexOf("}", 0);
                    while (start >= 0 && end > 0)
                    {
                        string str = json.Substring(start, end - start + 1);
                        System.Console.WriteLine(str);
                        T dataObj = (T)JsonConvert.DeserializeObject(str, typeof(T));
                        listData.Add(dataObj);
                        start = json.IndexOf("{", end);
                        end = json.IndexOf("}", end + 1);
                    }
                    return listData;
                }
                else
                    return null;
            }
        }
    }
}
