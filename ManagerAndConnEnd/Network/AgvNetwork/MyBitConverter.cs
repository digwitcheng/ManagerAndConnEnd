using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network
{
    class MyBitConverter
    {
        public static readonly bool IsLittleEndian = false;

        /// <summary>
        /// 返回由32位有符号整数转换为的字节数组
        /// </summary>
        /// <param name="value">整数</param>    
        /// <param name="endian">高低位</param>
        /// <returns></returns>
        public unsafe static byte[] GetBytes(int value)
        {
            byte[] bytes = new byte[4];
            fixed (byte* pbyte = &bytes[0])
            {
                if (IsLittleEndian)
                {
                    *pbyte = (byte)(value);
                    *(pbyte + 1) = (byte)(value >> 8);
                    *(pbyte + 2) = (byte)(value >> 16);
                    *(pbyte + 3) = (byte)(value >> 24);
                }
                else
                {
                    *(pbyte + 3) = (byte)(value);
                    *(pbyte + 2) = (byte)(value >> 8);
                    *(pbyte + 1) = (byte)(value >> 16);
                    *pbyte = (byte)(value >> 24);
                }
            }
            return bytes;
        }

        /// <summary>
        /// 返回由32位无符号整数转换为的字节数组
        /// </summary>
        /// <param name="value">整数</param>    
        /// <param name="endian">高低位</param>
        /// <returns></returns>
        public static byte[] GetBytes(uint value)
        {
            return GetBytes((int)value);
        }

        /// <summary>
        /// 返回由16位有符号整数转换为的字节数组
        /// </summary>
        /// <param name="value">整数</param>    
        /// <param name="endian">高低位</param>
        /// <returns></returns>
        public unsafe static byte[] GetBytes(short value)
        {
            byte[] bytes = new byte[2];
            fixed (byte* pbyte = &bytes[0])
            {
                if (IsLittleEndian)
                {
                    *pbyte = (byte)(value);
                    *(pbyte + 1) = (byte)(value >> 8);
                }
                else
                {
                    *(pbyte + 1) = (byte)(value);
                    *pbyte = (byte)(value >> 8);
                }
            }
            return bytes;
        }

        /// <summary>
        /// 返回由16位无符号整数转换为的字节数组
        /// </summary>
        /// <param name="value">整数</param>    
        /// <param name="endian">高低位</param>
        /// <returns></returns>
        public static byte[] GetBytes(ushort value)
        {
            return GetBytes((short)value);
        }
        public static unsafe short ToInt16(byte[] value,ref int startIndex)
        {
            if (value == null)
            {
                throw new ArgumentNullException("Toint16 value");
            }

            if ((uint)startIndex >= value[2])
            {
                throw new IndexOutOfRangeException("toint16 value length:" + value[2] + "startIndex:" + startIndex);
            }

            if (startIndex > value[2] - 2)
            {
                throw new ArgumentException("toint16 value length-2 must be greater than startIndex:"+startIndex);
            }

            fixed (byte* pbyte = &value[startIndex])
            {
                startIndex += 2;
                if (startIndex % 2 == 0)
                { // data is aligned 
                    return *((short*)pbyte);
                }
                else
                {
                    if (IsLittleEndian)
                    {
                        return (short)((*pbyte) | (*(pbyte + 1) << 8));
                    }
                    else
                    {
                        return (short)((*pbyte << 8) | (*(pbyte + 1)));
                    }
                }
            }

        }
        public static unsafe int ToInt32(byte[] value,ref int startIndex)
        {
            if (value == null)
            {
                throw new ArgumentNullException("ToInt32 value");
            }

            if ((uint)startIndex >= value[2])
            {
                throw new IndexOutOfRangeException("ToInt32 value length:" + value[2] + "startIndex:" + startIndex);
            }

            if (startIndex > value[2] - 4)
            {
                throw new ArgumentException("ToInt32 value length-2 must be greater than startIndex:" + startIndex);
            }

            fixed (byte* pbyte = &value[startIndex])
            {
                startIndex += 4;
                if (startIndex % 4 == 0)
                { // data is aligned 
                    return *((int*)pbyte);
                }
                else
                {
                    if (IsLittleEndian)
                    {
                        return (*pbyte) | (*(pbyte + 1) << 8) | (*(pbyte + 2) << 16) | (*(pbyte + 3) << 24);
                    }
                    else
                    {
                        return (*pbyte << 24) | (*(pbyte + 1) << 16) | (*(pbyte + 2) << 8) | (*(pbyte + 3));
                    }
                }
            }
            
        }

        public static ushort ToUInt16(byte[] value,ref int startIndex)
        {
            if (value == null)
            {
                throw new ArgumentNullException("Toint16 value");
            }

            if ((uint)startIndex >= value[2])
            {
                throw new IndexOutOfRangeException("toint16 value length:" + value[2] + "startIndex:" + startIndex);
            }

            if (startIndex > value[2] - 2)
            {
                throw new ArgumentException("toint16 value length-2 must be greater than startIndex:" + startIndex);
            }

            return (ushort)ToInt16(value,ref startIndex);
        }

        public static uint ToUInt32(byte[] value,ref int startIndex)
        {
            if (value == null)
            {
                throw new ArgumentNullException("ToInt32 value");
            }

            if ((uint)startIndex >= value[2])
            {
                throw new IndexOutOfRangeException("ToInt32 value length:" + value[2] + "startIndex:" + startIndex);
            }

            if (startIndex > value[2] - 4)
            {
                throw new ArgumentException("ToInt32 value length-2 must be greater than startIndex:" + startIndex);
            }

            return (uint)ToInt32(value,ref startIndex);
        }



    }
}
