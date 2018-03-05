using AGV_V1._0.NLog;
using AGV_V1._0.Util;
using Astar;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml;

namespace AGV_V1._0.Algorithm
{
    class FileUtil
    {
        public static XmlDocument xmlfile;
        public static SendData[] sendData;

        /// <summary>
        /// 初始化mapXML配置文件
        /// </summary>
        public static void LoadMapXml()
        {
            //XML1.0:获取xml文件路径
            string pathMap = ConstDefine.MAP_PATH;

            if (!File.Exists(pathMap))
            {
                throw new FileNotFoundException("mapXml");
            }
            xmlfile = new XmlDocument();
            xmlfile.Load(pathMap);
            //XML2.0:获取地图的格子数
            XmlNode map_w = xmlfile.SelectSingleNode("config/Map/Widthnum");
            XmlNode map_h = xmlfile.SelectSingleNode("config/Map/Heightnum");

            ConstDefine.g_WidthNum = Convert.ToInt32(map_w.InnerText);
            ConstDefine.g_HeightNum = Convert.ToInt32(map_h.InnerText);
            Logs.Info("load map success");


        }
        /// <summary>
        /// 初始化XML配置文件
        /// </summary>
        public static void LoadAgvXml()
        {

            //XML1.0:获取xml文件路径
            string pathAGV = ConstDefine.AGV_PATH ;
            if (!File.Exists(pathAGV))
            {
                throw new FileNotFoundException("agvxml");
            }
            XmlDocument xmlAgv = new XmlDocument();
            xmlAgv.Load(pathAGV);
            //XML2.0:获取vehicle数
            XmlNode agvsum = xmlAgv.SelectSingleNode("Info/SUM");
            XmlNode xmlNode = xmlAgv.SelectSingleNode("Info/AGV_info");
            if (xmlNode == null)
            {
                MessageBox.Show("约定的小车文件节点不存在");
                Logs.Fatal("约定的小车文件节点不存在,初始化Agv失败");
                throw new XmlException("agvxml");
            }
            XmlNodeList agvList = xmlNode.ChildNodes;
            if (agvList == null||agvList.Count<1)
            {
                Logs.Error("agvfile hasn't Info/AGV_infor node");
                throw new XmlException();
            }

            Int32 gridnum = Convert.ToInt32(agvsum.InnerText.ToString().Trim());
            Int32 realGridnum = agvList.Count;
            if (realGridnum < gridnum)
            {
                gridnum = realGridnum;
                string str = "agv文件中实际小车的数量小于SUM的数值，加载实际小车的数量";
                //  MessageBox.Show(str);
                Logs.Warn(str);
            }
            if (realGridnum > gridnum)
            {
                string str = "agv文件中实际小车的数量大于SUM的数值，加载小车的前SUM行";
                // MessageBox.Show(str);
                Logs.Warn(str);
            }
            ConstDefine.g_VehicleCount = gridnum;
            sendData = new SendData[gridnum];
            for (int i = 0; i < gridnum; i++)
            {
                int num = Convert.ToInt32(agvList[i].Attributes["Num"].InnerText.ToString());
                int x = Convert.ToInt32(agvList[i].Attributes["BeginX"].InnerText.ToString());
                int y = Convert.ToInt32(agvList[i].Attributes["BeginY"].InnerText.ToString());
                string state = agvList[i].Attributes["State"].InnerText.ToString();
                string battery = agvList[i].Attributes["Battery"].InnerText.ToString();
                sendData[i] = new SendData(num, x, y);
            }


        }


        /// <summary>
        /// 保存agv的信息
        /// </summary>
        /// <returns></returns>
        public static int SaveAgvInfo(Vehicle[] vehicle)
        {
            int res = -1;
            try
            {
                //XML1.0:获取xml文件路径
                string pathAGV = ConstDefine.AGV_PATH;
                if (!File.Exists(pathAGV))
                {
                    return res;
                }
                XmlDocument xmlAgv = new XmlDocument();
                xmlAgv.Load(pathAGV);
                //XML2.0:获取地图的格子数
                XmlNode agvsum = xmlAgv.SelectSingleNode("Info/SUM");
                XmlNode xmlNode = xmlAgv.SelectSingleNode("Info/AGV_info");
                if (xmlNode == null)
                {
                    MessageBox.Show("约定的小车文件节点不存在");
                    Logs.Error("约定的小车文件节点不存在");
                    return -1;
                }
                XmlNodeList agvList = xmlNode.ChildNodes;
                if (agvList == null)
                {
                    return 0;
                }
                Int32 gridnum = Convert.ToInt32(agvsum.InnerText.ToString().Trim());
                Int32 realGridnum = agvList.Count;
                if (realGridnum < gridnum)
                {
                    gridnum = realGridnum;
                    string str = "agv文件中实际小车的数量小于SUM的数值，加载实际小车的数量";
                    MessageBox.Show(str);
                    Logs.Warn(str);
                }
                if (realGridnum > gridnum)
                {
                    string str = "agv文件中实际小车的数量大于SUM的数值，加载小车的前SUM行";
                    MessageBox.Show(str);
                    Logs.Warn(str);
                }
                ConstDefine.g_VehicleCount = gridnum;
                for (int i = 0; i < gridnum; i++)
                {
                    agvList[i].Attributes["Num"].InnerText = vehicle[i].Id + "";
                    agvList[i].Attributes["BeginX"].InnerText = vehicle[i].BeginX + "";
                    agvList[i].Attributes["BeginY"].InnerText = vehicle[i].BeginY + "";
                    agvList[i].Attributes["State"].InnerText = vehicle[i].CurState.ToString();
                    agvList[i].Attributes["Battery"].InnerText = vehicle[i].Electricity + "";
                    agvList[i].Attributes["Direction"].InnerText = vehicle[i].Dir.ToString();
                    agvList[i].Attributes["StartLoc"].InnerText = vehicle[i].StartLoc.ToString();

                }
                xmlAgv.Save(pathAGV);
                MessageBox.Show("agvFile saved");
                Logs.Info("agvFile saved");
                return agvList.Count;
            }
            catch (System.Xml.XmlException xe)
            {
                Logs.Fatal("save avgFile failed:" + xe);
                return -1;
            }
        }


        /// <summary>  
        /// 获取指定节点的值  
        /// </summary>  
        /// <param name="strFileName">文件路径</param>  
        /// <param name="nodeName">节点名称</param>  
        /// <param name="value">设置后的值</param>  
        /// <param name="nodeDir">指定节点所在的节点目录</param>  
        /// <returns></returns>  
        public string GetNodeValue(string strFileName, string nodeName, string nodeDir)
        {
            string value = null;
            try
            {
                XmlDocument xmlDoc = new XmlDocument();
                xmlDoc.Load(strFileName);


                XmlNodeList nodeList = xmlDoc.SelectSingleNode(nodeDir).ChildNodes;//获取bookstore节点的所有子节点   


                foreach (XmlNode xn in nodeList)    //遍历所有子节点   
                {
                    XmlElement xe = (XmlElement)xn;  //将子节点类型转换为XmlElement类型   


                    if (xe.Name == nodeName)
                    {
                        value = xe.InnerText.Trim();

                        break;
                    }
                }
            }
            catch (Exception exp)
            {
                throw;
            }
            return value;
        }

    }
}
