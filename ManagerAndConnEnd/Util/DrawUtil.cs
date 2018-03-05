using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Util
{
    class DrawUtil
    {
        public static void FillRectangle(Graphics g, Color color, int x, int y, int width, int height)
        {
            g.FillRectangle(new SolidBrush(color), new Rectangle(x, y, width,height));            
        }
        public static void FillRectangle(Graphics g, Color color, Rectangle rect)
        {
            g.FillRectangle(new SolidBrush(color), rect);
        }
       
        public static void DrawString(Graphics g, int context, float fontSize, Color fontColor, int x, int y)
        {
            DrawString(g, context+"", fontSize, fontColor, x, y);
        }
        

        public static void DrawString(Graphics g, string context, float fontSize, Color fontColor, int x, int y)
        {
            DrawString(g, context, fontSize, fontColor, new PointF(x, y));
        }

        public static void DrawString(Graphics g, int context, float fontSize, Color fontColor, PointF pf)
        {
            DrawString(g, context + "", fontSize, fontColor, pf);
        }
        public static void DrawString(Graphics g, string context, float fontSize, Color fontColor, PointF pf)
        {
            Font font = new Font(new System.Drawing.FontFamily("宋体"), fontSize);
            Brush brush = new SolidBrush(fontColor);
            g.DrawString(context + "", font, brush, pf);
        }
    }
}
