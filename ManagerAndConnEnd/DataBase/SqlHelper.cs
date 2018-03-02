using System;
using System.Data.SqlClient;
using System.Data;
using AGV_V1._0.NLog;

namespace DataBase
{
    /// <summary>
    /// SqlServer数据库操作相关类
    /// </summary>
    public class SqlHelper
    {
        //public static string LocalDbConnStr = "";
        //Data Source=DIGWITC;Initial Catalog=agv;Integrated Security=True
        public static string CONECTIONG_STRING = "Data Source=LocalHost;Initial Catalog=agv;User Id=sa;Password=1046541763;";

        #region 获得一个本地数据库(SQLSERVER2008)的SQL连接
        public static SqlConnection GetSqlConnection()
        {
            SqlConnection sqlConn = new SqlConnection();
            sqlConn.ConnectionString = SqlHelper.CONECTIONG_STRING;

            if (SqlHelper.OpenSqlConn(ref sqlConn) == true)
            {
                return sqlConn;
            }
            else
            {
                return null;
            }
        }
        #endregion

        #region 判断某个Sql连接是否打开
        public static bool IsOpenedConn(SqlConnection sqlconn)
        {
            if (sqlconn == null)
            {
                return false;
            }

            try
            {
                if (sqlconn.State == ConnectionState.Open)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            catch
            {
                return false;
            }
        }
        #endregion

        #region 打开某个SQL连接
        public static bool OpenSqlConn(ref SqlConnection sqlconn)
        {
            if (sqlconn == null)
            {
                return false;
            }

            try
            {
                if (sqlconn.State != ConnectionState.Open)
                {
                    sqlconn.Open();
                }
                else
                {
                    return true;
                }

                if (sqlconn.State == ConnectionState.Open)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            catch (Exception ex)
            {
                Logs.Error("SqlHelper.OpenSqlConn" + "SQL:打开数据库连接时捕获异常:" + ex.Message);
                return false;
            }
        }
        #endregion

        #region 关闭某个SQL连接
        public static bool CloseSqlConn(ref SqlConnection sqlconn)
        {
            if (sqlconn == null)
            {
                return true;
            }

            try
            {
                if (sqlconn.State == ConnectionState.Open)
                {
                    sqlconn.Close();

                    return true;
                }
                else
                {
                    return true;
                }
            }
            catch (Exception ex)
            {
                Logs.Error("SqlHelper.CloseSqlConn " + "SQL:关闭数据库连接时捕获异常:" + ex.Message);
                return false;
            }
        }
        #endregion

        #region 执行SQL语句
        public static int ExecNonQuery(SqlConnection sqlconn, string cmdTxt)
        {
            int rowCount = 0;
            SqlCommand cmd = null;

            if (sqlconn == null)
            {
                throw new ArgumentNullException("sqlConnection");
            }

            if (sqlconn.State == ConnectionState.Closed)
            {
                if (SqlHelper.OpenSqlConn(ref sqlconn) == false)
                {
                    return 0;
                }
            }
            cmd = new SqlCommand(cmdTxt, sqlconn);
                try
                {

                    cmd.CommandType = CommandType.Text;
                    cmd.CommandTimeout = 10;

                    rowCount = cmd.ExecuteNonQuery();
                }
                catch (Exception ex)
                {
                    Logs.Error("SqlHelper.ExecNonQuery " + "SQL:SqlHelper.ExecNonQuery方法捕获异常:" + ex.Message);

                    rowCount = 0;
                }
                finally
                {
                    cmd.Dispose();
                
            }

            return rowCount;
        }
        #endregion

        #region 返回首行首列
        public static object ExecScalar(SqlConnection sqlconn, string cmdTxt)
        {
            object objResult = null;
            SqlCommand cmd = null;

            if (sqlconn == null)
            {
                throw new ArgumentNullException("sqlConnection");
            }

            if (sqlconn.State == ConnectionState.Closed)
            {
                if (SqlHelper.OpenSqlConn(ref sqlconn) == false)
                {
                    return null;
                }
            }

            try
            {
                cmd = new SqlCommand(cmdTxt, sqlconn);
                cmd.CommandType = CommandType.Text;
                cmd.CommandTimeout = 10;

                objResult = cmd.ExecuteScalar();
            }
            catch (Exception ex)
            {
                Logs.Error("SqlHelper.ExecScalar " + "SQL:SqlHelper.ExecScalar方法捕获异常:" + ex.Message);

                objResult = null;
            }
            finally
            {
                cmd.Dispose();
            }

            return objResult;
        }
        #endregion

        #region PrepareCommand
        private static void PrepareCommand(SqlCommand sqlCommand, SqlParameter[] commandParms)
        {
            if (sqlCommand == null)
            {
                throw new ArgumentNullException("PrepareCommand sqlCommand");
            }
            if (commandParms != null)
            {
                foreach (SqlParameter parameter in commandParms)
                {
                    if (parameter.Value == null)
                    {
                        parameter.Value = DBNull.Value;
                    }
                    sqlCommand.Parameters.Add(parameter);
                }
            }
        }
        #endregion

        #region 执行存储过程
        public static bool ExecProcedure(SqlConnection sqlconn, string procedureName, ref SqlParameter[] cmdParams)
        {
            bool bResult = false;
            SqlCommand cmd = null;

            if (sqlconn == null)
            {
                throw new ArgumentNullException("ExecProcedure sqlconn");
            }

            if (sqlconn.State == ConnectionState.Closed)
            {
                if (SqlHelper.OpenSqlConn(ref sqlconn) == false)
                {
                    return false;
                }
            }

            try
            {
                cmd = new SqlCommand();
                {
                    cmd.CommandType = CommandType.StoredProcedure;
                    cmd.CommandTimeout = 10;
                    cmd.CommandText = procedureName;  //存储过程名称
                    cmd.Connection = sqlconn;

                    PrepareCommand(cmd, cmdParams);

                    cmd.ExecuteNonQuery();

                    bResult = true;
                }
            }
            catch (Exception ex)
            {
                Logs.Error("SqlHelper.ExecProcedure " + "SQL:SqlHelper.ExecProcedure方法捕获异常:" + ex.Message);

                bResult = false;
            }
            finally
            {
                cmd.Dispose();
            }

            return bResult;
        }
        #endregion

        #region 返回一个数据集    
           private static SqlDataAdapter sda = new SqlDataAdapter();
           private static DataSet ds = new DataSet();
          // private static SqlCommand myCmd = new SqlCommand();
         //  private static DataTable myTable = new DataTable();
       // private static DataTable ds = new DataTable();
        public static DataSet GetDataSet(SqlConnection sqlconn, string cmdTxt)
        {
          // SqlDataAdapter sda = null;
          //  DataSet ds = null;

            if (sqlconn == null)
            {
                throw new ArgumentNullException("GetDataSet sqlconn");
            }

            if (sqlconn.State == ConnectionState.Closed)
            {
                if (SqlHelper.OpenSqlConn(ref sqlconn) == false)
                {
                    return null;
                }
            }
            using (sda = new SqlDataAdapter(cmdTxt, sqlconn))
            {
                try
                {
                    ds = new DataSet();

                    sda.Fill(ds);
                }
                catch (Exception ex)
                {
                    Logs.Error("SqlHelper.GetDataSet " + "SQL:SqlHelper.GetDataSet方法捕获到异常:" + ex.Message);

                    ds = null;
                }
                finally
                {
                    sda.Dispose();
                }
            }

            return ds;
        }
        #endregion

        #region 返回一个数据表
        public static DataTable GetDataTable(SqlConnection sqlconn, string cmdTxt)
        {
            //
           // DataSet ds = SqlHelper.GetDataSet(sqlconn, cmdTxt);
            ds = SqlHelper.GetDataSet(sqlconn, cmdTxt);

            if (ds == null || ds.Tables.Count < 1)
            {
                return null;
            }

            return ds.Tables[0] as DataTable;
        }
        #endregion

    }
}
