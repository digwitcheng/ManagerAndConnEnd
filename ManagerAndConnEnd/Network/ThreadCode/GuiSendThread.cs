using AGV_V1._0.Event;
using AGV_V1._0.Util;
using CowboyTest.Server.APM;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AGV_V1._0.Network.ThreadCode
{
    class GuiSendThread:BaseThread
    {
        private static GuiSendThread instance;
        public static GuiSendThread Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new GuiSendThread();
                }
                return instance;
            }
        }
        private GuiSendThread()
        {
        }
        protected override string ThreadName()
        {
            return "GuiSend";
        }
        protected override void Run()
        {
            try
            {
                SendDataToGui();
                Thread.Sleep(ConstDefine.GUI_TIME);
            }
            catch (Exception ex)
            {
                OnShowMessage(this, new MessageEventArgs(ex.Message));
            }
        }
         void SendDataToGui()
        {
            try
            {

                Vehicle[] vehicle = VehicleManager.Instance.GetVehicles();
                if (vehicle != null)
                {
                    string data = JsonHelper.VehiclesToGuiJson(vehicle);
                    GuiServerManager.Instance.Broadcast(MessageType.Msg, data);
                }

            }
            catch { throw; }
        }
    }
}
