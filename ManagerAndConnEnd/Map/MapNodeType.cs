using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using AGV_V1._0.Properties;
using System.Drawing;

namespace AGV_V1._0
{
        enum MapNodeType
        {
            None,                  //0无
            Road,                  //1道路
            queuingArea,           //2排队区
            queueEntra,            //3排队区入口
            scanner,               //4扫描仪
            restArea,              //5休息区
            chargingArea,          //6充电区
            dropBag,               //7下包位（暂无）

            deliveryPort,          //8投放口
            obstacle,              //9障碍
            platform,              //10工位
            forbidNode             //11禁入点
        }
}
