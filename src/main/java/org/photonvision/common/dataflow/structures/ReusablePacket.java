package org.photonvision.common.dataflow.structures;

/**
 * Extension of Packet that allows zero-allocation reuse. By placing this in the same package as
 * Packet, we can access package-private fields without reflection.
 */
public class ReusablePacket extends Packet {
	public ReusablePacket(int size) {
		super(size);
	}

	/**
	 * Resets read and write positions to 0 without allocating a new byte array. This allows the
	 * packet to be reused across multiple encode/decode cycles without any allocations.
	 */
	public void reset() {
		readPos = 0;
		writePos = 0;
	}

	/**
	 * Replaces the internal byte array reference without copying. Use this to update the packet
	 * data for decoding without allocation.
	 *
	 * @param data The new byte array to use
	 */
	public void setData(byte[] data) {
		packetData = data;
		readPos = 0;
		writePos = 0;
	}

	/**
	 * Returns a direct reference to the internal byte array (not a copy). The returned array may be
	 * larger than the actual written data - use getNumBytesWritten() to determine the valid range.
	 * WARNING: This avoids allocation but shares the internal buffer. Do not modify the returned
	 * array.
	 *
	 * @return Direct reference to the internal packet buffer
	 */
	public byte[] getDataReference() {
		return packetData;
	}
}
