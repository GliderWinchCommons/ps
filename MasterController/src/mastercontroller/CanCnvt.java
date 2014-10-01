/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package mastercontroller;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import javax.xml.bind.DatatypeConverter;
import java.lang.Math;

/**
 * CAN message
 *
 * Handle the conversion between binary and ascii/hex format CAN messages.
 *
 * @author deh
 */
/*
 *   Decode methods for extracting data from payload extneded by
 *   @author gsm
 *
 */
public class CanCnvt
{

    public int val; // Valid indicator (zero if valid)

    public int seq; // Sequence number (if used)
    /**
     * 32b word with CAN id (STM32 CAN format)
     */
    public int id;  // 32b word with CAN id (STM32 CAN format)
    public int dlc; // Payload count (number of bytes in payload)

    public byte[] pb;// Binary bytes as received and converted from ascii/hex input
    public int chk; // Byte checksum

    /* *********************************************************************
     * Constructors
     * ********************************************************************* */
    public CanCnvt()
    {
        pb = new byte[15];
    }

    public CanCnvt(int iseq, int iid)
    {
        seq = iseq;
        id = iid;
        pb = new byte[15];
    }

    public CanCnvt(int iseq, int iid, int idlc)
    {
        seq = iseq;
        id = iid;
        dlc = idlc;
        pb = new byte[15];

    }

    public CanCnvt(int iseq, int iid, int idlc, byte[] px)
    {
        seq = iseq;
        id = iid;
        dlc = idlc;
        pb = new byte[15];
        pb = px;

    }
    /* *************************************************************************
     Compute CAN message checksum on binary array
     param   : byte[] b: Array of binary bytes in check sum computation\
     param   : int m: Number of bytes in array to use in computation
     return  : computed checksum
     ************************************************************************ */
    private byte checksum(int m)
    {
        int chktot = 0xa5a5;    // Initial value for computing checksum
        for (int i = 0; i < m; i++)
        {
            chktot += (pb[i] & 0xff);// Build total (int) from signed byte array
        }
        /* Add in carries and carry from adding carries */
        chktot += (chktot >> 16); // Add carries from low half word
        chktot += (chktot >> 16); // Add carry from above addition
        chktot += (chktot >> 8);  // Add carries from low byte
        chktot += (chktot >> 8);  // Add carry from above addition
        return (byte) chktot;
    }

    /**
     * Check message for errors and Convert incoming ascii/hex CAN msg to an
     * array of bytes plus assemble the bytes comprising CAN ID into an int.
     * @param msg: String with CAN message
     * @return codes:
     *  0 = OK;
     * -1 = message too short (less than 14)
     * -2 = message too long (greater than 30)
     * -3 = number of bytes not even
     * -4 = payload count is negative or greater than 8
     * -5 = checksum error
     * -6 = non-ascii/hex char in input
     */
    public int convert_msgtobin(String msg)
    {
        int m = msg.length();
        if (m < 14)
        {
            return -1;  // Too short for a valid CAN msg
        }
        if (m > 30)
        {
            return -2;  // Longer than the longest CAN msg
        }
        if ((m & 0x1) != 0)
        {
            return -3; // Not even: asci1-hex must be in pairs
        }
        try{
            pb = DatatypeConverter.parseHexBinary(msg); // Convert ascii/hex to byte array
        }
        catch(IllegalArgumentException e){
            System.err.println("Caught IOException: " + e.getMessage());
            return -6;
        }

        /* Check computed checksum versus recieved checksum.  */
        byte chkx = checksum((m / 2) - 1);
        if (chkx != pb[((m / 2) - 1)])
        {
            // System.out.println(msg);    // Display for debugging
            // for (int j = 0; j < (m / 2); j++)
            // {
            //     System.out.format("%02X ", pb[j]);
            // }
            // System.out.format("chkx: %02X" + " pb[((m/2) -1)]: %02X\n", chkx,
            //         pb[((m / 2) - 1)]);
            return -5; // Return error code
        }

        /* Check that the payload count is within bounds */
        if (pb[5] < 0)
        {
            return -4;    // This should not be possible
        }
        if (pb[5] > 8)
        {
            return -4;    // Too large means something wrong.
        }
        /* Extract some items that are of general use */
        seq = (pb[0] & 0xff);     // Sequence number byte->unsigned
        dlc = (pb[5] & 0xff);     // Save payload ct in an easy to remember variable
        id = ((((((pb[4] << 8) | (pb[3] & 0xff)) << 8)
                | (pb[2] & 0xff)) << 8) | (pb[1] & 0xff));
        return 0;
    }

    /**
     * Extract long from payload
     *
     * @return payload bytes [0]-[7] as one long
     */
    public long get_long()
    {
        if (pb[5] != 8)
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            int x0 = ((((((pb[ 9] << 8) | (pb[ 8] & 0xff)) << 8)
                    | (pb[ 7] & 0xff)) << 8) | (pb[ 6] & 0xff));
            int x1 = ((((((pb[13] << 8) | (pb[12] & 0xff)) << 8)
                    | (pb[11] & 0xff)) << 8) | (pb[10] & 0xff));
            // Combine to make a long
            long lng = ((long) x1 << 32) | (x0 & 0xffffffffL);
            val = 0;
            return lng;
        }
    }

    /**
     * Extract four payload bytes to an int,
     *
     * @param offset of first byte in payload (0 - 4)
     * @return int, val = -1 if insufficient payload bytes
     */
    public int get_int(int offset)
    {
        if (pb[5] < (offset + 4))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            int nt = ((((((pb[offset + 9] << 8)
                    | (pb[offset + 8] & 0xff)) << 8)
                    | (pb[offset + 7] & 0xff)) << 8)
                    | (pb[offset + 6] & 0xff));
            val = 0;
            return nt;
        }
    }

    /**
     * Convert bytes to an int array
     *
     * @param offset is offset from start of payload
     * @param numb is number of ints to convert
     * @return int[] of values, val = -1 if insufficient payload bytes
     */
    public int[] get_ints(int offset, int numb)
    {
        if (pb[5] < offset + numb * 4)
        {
            val = -1;   // insufficient payload length;
            return new int[0];
        } else
        {
            int[] nt = new int[numb];
            for (int i = 0; i < numb; i++)
            {
                int i4o = i * 4 + offset;
                nt[i] = ((((((pb[i4o + 9] << 8)
                        | (pb[i4o + 8] & 0xff)) << 8)
                        | (pb[i4o + 7] & 0xff)) << 8)
                        | (pb[i4o + 6] & 0xff));
            }
            val = 0;
            return nt;
        }
    }

    /**
     * Combine four payload bytes to an uint
     *
     * @param offset of first byte in payload (0 - 4)
     * @return long to hold full range, val = -1 if insufficient payload bytes
     */
    public long get_uint(int offset)
    {
        if (pb[5] < (offset + 4))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            long lng;
            lng = ((long) (((((pb[offset + 9] & 0xff) << 8)
                    | (pb[offset + 8] & 0xff)) << 8)
                    | (pb[offset + 7] & 0xff)) << 8)
                    | (pb[offset + 6] & 0xff);
            val = 0;
            return lng;
        }
    }

    /**
     *
     * Extract short from payload
     *
     * @param offset range (0 - 6)
     * @return short, val = -1 if insufficient payload bytes
     */
    public short get_short(int offset)
    {
        short st;
        if (pb[5] < (offset + 2))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            st = (short) ((pb[offset + 7] << 8)
                    | (pb[offset + 6] & 0xff));
        }
        val = 0;
        return st;

    }

    /**
     *
     * Extract shorts from payload
     *
     * @param offset range (0 - 6)
     * @param numb number of shorts (0-4)
     * @return short[], length 0 with val = -1 if insufficient payload bytes
     */
    public short[] get_shorts(int offset, int numb)
    {
        if (pb[5] < offset + numb * 2)
        {
            val = -1;   // insufficient payload length
            return new short[0];
        } else
        {
            short[] sh = new short[numb];
            for (int i = 0; i < numb; i++)
            {
                int i2o = i * 2 + offset;
                sh[i] = (short) ((pb[i2o + 7] << 8)
                        | (pb[i2o + 6] & 0xff));
            }
            val = 0;
            return sh;
        }
    }

    /**
     *
     * Extract ushort from payload
     *
     * @param offset range (0 - 6)
     * @return int, val = -1 if insufficient payload bytes
     */
    public int get_ushort(int offset)
    {
        if (pb[5] < (offset + 2))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            int ust = ((pb[offset + 7] & 0xff) << 8)
                    | (pb[offset + 6] & 0xff);
            return ust;
        }
    }

    /**
     *
     * Extract shorts from payload
     *
     * @param offset range (0 - 6)
     * @param numb number of shorts (0-4)
     * @return short[], length 0 with val = -1 if error
     */
    public int[] get_ushorts(int offset, int numb)
    {
        int[] ust = new int[numb];
        if (pb[5] < offset + numb * 2)
        {
            val = -1;   // insufficient payload length
            return new int[0];
        } else
        {

            for (int i = 0; i < numb; i++)
            {
                int i2o = i * 2 + offset;
                ust[i] = ((pb[i2o + 7] & 0xff) << 8)
                        | (pb[i2o + 6] & 0xff);

            }
        }
        val = 0;
        return ust;

    }

    /**
     *
     * Extract byte from payload
     *
     * @param offset range (0 - 7)
     * @return byte, 0 with val = -1 if insufficient payload
     */
    public byte get_byte(int offset)
    {
        if (pb[5] < (offset + 1))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            byte by = pb[offset + 6];
            val = 0;
            return by;
        }
    }

    /**
     *
     * Extract bytes from payload
     *
     * @param offset range (0 - 7)
     * @param numb number of shorts (0 - 4)
     * @return short[], length 0 with val = -1 if insufficient payload bytes
     */
    public byte[] get_bytes(int offset, int numb)
    {
        if (pb[5] < offset + numb)
        {
            val = -1;   // insufficient payload length
            return new byte[0];
        } else
        {
            byte[] by = new byte[numb];
            for (int i = 0; i < numb; i++)
            {
                by[i] = pb[offset + 6];
            }
            val = 0;
            return by;
        }
    }

    /**
     *
     * Extract ubytes from payload
     *
     * @param offset range (0 - 7)
     * @return short, with val = -1 if insufficient payload
     */
    public short get_ubyte(int offset)
    {
        if (pb[5] < (offset + 1))
        {
            val = -1;   // insufficient payload length
            return 0;
        } else
        {
            return (short) (pb[offset + 6] & 0xff);
        }
    }

    /**
     *
     * Extract ubytes from payload
     *
     * @param offset range (0 - 7)
     * @param numb number of shorts (0-4)
     * @return short[], length 0 with val = -1 if insufficient payload bytes
     */
    public short[] get_ubytes(int offset, int numb)
    {
        if (pb[5] < offset + numb)
        {
            val = -1;   // insufficient payload length
            return new short[0];
        } else
        {
            short[] sh = new short[numb];
            for (int i = 0; i < numb; i++)
            {
                int ipo = offset + i;
                sh[i] = (short) (pb[ipo + 6] & 0xff);
            }
            val = 0;
            return sh;
        }
    }

    /**
     * Convert bytes to an uint array
     *
     * @param offset is offset from start of payload
     * @param numb is number of ints to convert
     * @return long[] to hold full range, length 0 if error
     */
    public long[] get_uints(int offset, int numb)
    {
        if (pb[5] < offset + numb * 4)
        {
            val = -1;   // insufficient payload length
            return new long[0];
        } else
        {
            long[] lng = new long[numb];
            for (int i = 0; i < numb; i++)
            {
                int i4o = offset + i * 4;
                lng[i] = ((long) (((((pb[i4o + 9] & 0xff) << 8)
                        | (pb[i4o + 8] & 0xff)) << 8)
                        | (pb[i4o + 7] & 0xff)) << 8)
                        | (pb[i4o + 6] & 0xff);
            }
            val = 0;
            return lng;
        }
    }

    /**
     * Prepare CAN msg: Convert the array pb[] to hex and add checksum
     * The binary array pb[] is expected to have been set up.
     *
     * @return String with ascii/hex in ready to send
     *   return = null for error (dlc length out-of-range)
     */
    public String msg_prep()
    {  // Convert payload bytes from byte array

        /* A return of 'null' indicates an error */
        if (dlc > 8)
        {
            return null;
        }
        if (dlc < 0)
        {
            return null;
        }

        pb[0] = (byte)(seq);    // Set sequence number

        
        /* Setup Id bytes, little endian */
        pb[1] = (byte) (id);
        pb[2] = (byte) ((id >> 8));
        pb[3] = (byte) ((id >> 16));
        pb[4] = (byte) ((id >> 24));

        pb[5] = (byte) dlc;    // Payload size

        int msglength = (dlc + 6); // Length not including checksum

        pb[(msglength)] = checksum(msglength); // Place checksum in array

        /* Convert binary array to ascii/hex */
        StringBuilder x = new StringBuilder(DatatypeConverter.printHexBinary(pb));
        x.delete( ((msglength + 1)*2), 31);
        x.append("\n"); // Line terminator
        return x.toString();
    }
    
    /**
     * Big endian int to be stored in byte array little endian
     * Convert to payload byte array, little endian
     * @param n = int to be converted
     * @param offset = array index into payload (0 - 7)
     * @return 0 = OK; -1 =  error
     */
    public int set_int(int n, int offset)
    {
        if (offset > 4)
        {
            return -1;
        } else
        {
            pb[6 + offset] = (byte) (n);
            pb[7 + offset] = (byte) ((n >> 8));
            pb[8 + offset] = (byte) ((n >> 16));
            pb[9 + offset] = (byte) ((n >> 24));
            dlc = Math.max(dlc, offset + 4);
            return 0;
        }
    }

    /**
     * Big endian ints  to be stored in byte array little endian
     * Convert to payload byte array little endian offset
     * @param ns = int array to be converted
     * @param offset = array index into payload (0 - 7)
     * @return 0 = OK; -1 =  error
     */
    public int set_ints(int[] ns, int offset)
    {
        if (ns.length * 4 + offset > 8)
        {
            return - 1;
        } else
        {
            for (int i = 0; i < ns.length; i++)
            {
                int i4o = i * 4 + offset;
                pb[6 + i4o] = (byte) (ns[i]);
                pb[7 + i4o] = (byte) (ns[i] >> 8);
                pb[8 + i4o] = (byte) (ns[i] >> 16);
                pb[9 + i4o] = (byte) (ns[i] >> 24);
            }
            dlc = Math.max(dlc, ns.length * 4 + offset);
            return 0;
        }
    }

    /**
     * Big endian long to be stored in byte array little endian
     * Convert to payload byte array little endian offset
     * dlc is set to 8;
     * @param lng = long to be converted
     * @return 0 = OK; -1 =  error
     * *********************************************************************
     */
    public int set_long(long lng)
    {
        pb[ 6] = (byte) (lng);
        pb[ 7] = (byte) ((lng >> 8));
        pb[ 8] = (byte) ((lng >> 16));
        pb[ 9] = (byte) ((lng >> 24));
        pb[10] = (byte) ((lng >> 32));
        pb[11] = (byte) ((lng >> 40));
        pb[12] = (byte) ((lng >> 48));
        pb[13] = (byte) ((lng >> 56));
        dlc = 8;
        return 0;
    }

    /**
     *
     * @param n value to be converted: int for holding an unsigned short.
     * @param offset payload array offset (0 - 7)
     * @return 0 = OK; -1 = error;
     */
    public int set_short(int n, int offset)
    {
        if (offset > 6 )
        {
            return -1;
        } else
        {
            pb[6 + offset] = (byte) (n);
            pb[7 + offset] = (byte) ((n >> 8));
            dlc = Math.max(dlc, offset + 2);
            return 0;
        }
    }

    /**
     * Convert int array into payload shorts
     * @param ns  int array: value to be converted: int for holding an unsigned short
     * @param offset payload array offset (0 - 7)
     * @return 0 = OK; -1 = error;
     */
    public int set_shorts(int[] ns, int offset)
    {
        if (ns.length * 2 + offset > 8)
        {
            return - 1;
        } else
        {
            for (int i = 0; i < ns.length; i++)
            {
                int i2o = i * 2 + offset;
                pb[6 + i2o] = (byte) (ns[i]);
                pb[7 + i2o] = (byte) (ns[i] >> 8);
                pb[8 + i2o] = (byte) (ns[i] >> 16);
                pb[9 + i2o] = (byte) (ns[i] >> 24);
            }
            dlc = Math.max(dlc, ns.length * 2 + offset);
            return 0;
        }
    }

    /**
     * Place a byte into the payload
     * @param n low order byte of 'n' is placed
     * @param offset payload array offset (0 - 7)
     * @return 0 = OK; -1 = error;
     */
    public int set_byte(int n, int offset)
    {
        if (offset > 7 )
        {
            return -1;
        } else
        {
            pb[6 + offset] = (byte) (n);
            dlc = Math.max(dlc, offset + 1);
            return 0;
        }
    }

    /**
     * Place bytes from an array into payload
     * @param ns  Low order byte of each int in array
     * @param offset payload array offset (0 - 7)
     * @return 0 = OK; -1 = error;
     */
    public int set_bytes(int[] ns, int offset)
    {
        if (ns.length + offset > 8)
        {
            return - 1;
        } else
        {
            for (int i = 0; i < ns.length; i++)
            {
                int ipo = i + offset;
                pb[6 + ipo] = (byte) (ns[i]);
            }
            dlc = Math.max(dlc, ns.length + offset);
            return 0;
        }
    }



    /* Debugging code: calling routine calls this to check 'val' */
    public void valerr()
    {
        if (val != 0)
        {
            System.out.format("CanCnvt val err: %d\n", val);
        }
    }

}