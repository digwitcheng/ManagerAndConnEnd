using AGV_V1._0.Util;
using NLog;
using NLog.Config;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.NLog
{
    /// <summary>
    /// 项目日志封装
    /// </summary>
    public class Logs
    {
        private static Logger logger = LogManager.GetCurrentClassLogger(); //初始化日志类


        /// <summary>
        /// 静态构造函数
        /// </summary>
        static Logs()
        {
            //初始化配置日志
            LogManager.Configuration = new XmlLoggingConfiguration(System.AppDomain.CurrentDomain.BaseDirectory.ToString() + ConstDefine.CONFIG_PATH);
        }
        /// <summary>
        /// 调试日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        public static void Debug(String msg)
        {
            logger.Debug(msg);
        }

        /// <summary>
        /// 信息日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        /// <remarks>
        ///     适用大部分场景
        ///     1.记录日志文件
        /// </remarks>
        public static void Info(String msg)
        {
            logger.Info(msg);
        }

        /// <summary>
        /// 错误日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        /// <remarks>
        ///     适用异常,错误日志记录
        ///     1.记录日志文件
        /// </remarks>
        public static void Error(String msg)
        {
             Error(msg, null);        
        }

        /// <summary>
        /// 错误日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        /// /// <param name="ex">异常</param>
        /// <remarks>
        ///     适用异常,错误日志记录
        ///     1.记录日志文件
        /// </remarks>

        public static void Error(String msg, Exception ex)
        {
            logger.Error(msg,ex);
        }

        /// <summary>
        /// 严重致命错误日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        /// <remarks>
        ///     1.记录日志文件
        ///     2.控制台输出
        /// </remarks>
        public static void Fatal(String msg)
        {
            logger.Fatal(msg);
        }

        /// <summary>
        /// 警告日志
        /// </summary>
        /// <param name="msg">日志内容</param>
        /// <remarks>
        ///     1.记录日志文件
        ///     2.发送日志邮件
        /// </remarks>
        public static void Warn(String msg)
        {
            try
            {
                logger.Warn(msg);
            }
            catch { }
        }
    }
}
