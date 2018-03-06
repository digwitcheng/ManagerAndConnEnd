using AGV_V1._0.Algorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


namespace Agv.PathPlanning
{
    class Open
    {
      public  int length;        //当前队列的长度
      public   Close[] Array = new Close[AstarUtil.MaxLength];    //评价结点的指针
    }
}
