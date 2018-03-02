using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network.MyException
{
    enum ExceptionCode:byte
    {
        /// <summary>
        /// 未知错误
        /// </summary>
        UnknowError=0x01,

        /// <summary>
        /// 校验出错
        /// </summary>
        CheckSumError = 0X02,

        /// <summary>
        /// 不支持该类型
        /// </summary>
        NonsupportType = 0X03,

        /// <summary>
        /// 不支持该操作
        /// </summary>
        NonsupportOperation = 0X04,

        /// <summary>
        /// 数据缺失
        /// </summary>
        DataMiss=0x05
    }
}
