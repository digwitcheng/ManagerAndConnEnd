using System;
using System.Collections.Generic;
using System.Data;
using System.Data.SqlClient;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.DataBase
{
    class Sql
    {
        public static string connectionString = "Data Source=LocalHost;Initial Catalog=agv;User Id=sa;Password=1046541763;";
        // public static SqlConnection conn;    //隐藏这一行 
        public static SqlConnection Connection
        {
            get
            {

                // string connectionString = ConfigurationManager.ConnectionStrings["sql"].ToString();

                SqlConnection conn = new SqlConnection(connectionString);//在这里新建一个
                if (conn.State == System.Data.ConnectionState.Closed)//增加一个判断语句
                {
                    conn.Close();
                    conn.Open();
                }
                if (conn == null)
                {
                    conn.Open();
                }
                else if (conn.State == System.Data.ConnectionState.Closed)
                {
                    conn.Open();
                }
                else if (conn.State == System.Data.ConnectionState.Broken)
                {
                    conn.Close();
                    conn.Open();
                }
                return conn;
            }
        }
        public static DataTable GetDataSet(string safeSql)
        {
            DataSet ds = new DataSet();
            using (SqlCommand cmd = new SqlCommand(safeSql, Connection))
            {
                SqlDataAdapter da = new SqlDataAdapter(cmd);
                da.Fill(ds);
            }
            return ds.Tables[0];

        }
        //执行(增、删、改)返回受影响的行数
        public static int ExecuteCommand(string safeSql)
        {
            SqlCommand cmd = new SqlCommand(safeSql, Connection);
            int result = cmd.ExecuteNonQuery();
            return result;
        }
    }
}
