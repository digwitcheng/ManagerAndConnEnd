using AGV_V1._0.Util;
using Agv.PathPlanning;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.DataBase
{
    class DataBseUtil
    {
        public static bool EqualWithSqlCurLoction(int id, int x, int y)
        {
            MyPoint curPoint = SqlManager.Instance.GetVehicleCurLocationWithId(id);
            if (curPoint != null)
            {
                if (Math.Abs(x - curPoint.X) < ConstDefine.DEVIATION && Math.Abs(y - curPoint.Y) < ConstDefine.DEVIATION)
                {
                    return true;
                }
                else
                {
                    return false;
                }

            }
                //
            else
            {
                return false;
            }
        }        
        public static bool EqualWithSqlDesLoction(int id, int x, int y)
        {
            MyPoint desPoint = SqlManager.Instance.GetVehicleDesLocationWithId(id);
            if (desPoint != null)
            {
                if (Math.Abs(x - desPoint.X) < ConstDefine.DEVIATION && Math.Abs(y - desPoint.Y) < ConstDefine.DEVIATION)
                {
                    return true;
                }
                else { return false; }

            }
            else
            {
                return false;
            }
        }
        public static bool ReallyArrived(int id)
        {
            MyPoint curPoint = SqlManager.Instance.GetVehicleCurLocationWithId(id);
            MyPoint desPoint = SqlManager.Instance.GetVehicleDesLocationWithId(id);
            if (desPoint != null&&curPoint!=null)
            {
                if (Math.Abs(curPoint.X - desPoint.X) < ConstDefine.DEVIATION && Math.Abs(curPoint.Y - desPoint.Y) < ConstDefine.DEVIATION)
                {
                    return true;
                }
                else
                {
                 return false;
                }

            }
            else
            {
                return false;
            }
        }
    }
}
