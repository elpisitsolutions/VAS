import SAMBA 3.9
import SAMBA.Connection.Serial 3.9
import SAMBA.Device.SAMA5D4 3.9

SerialConnection {
	port: "COM4"

	device: SAMA5D4Xplained {
		//config {
		//	serialflash {
		//		ioset: 1
		//		busWidth: 8
		//		header: 0xc1e04e07
		//	}
		//}
	}

	onConnectionOpened: {
		// initialize Low-Level applet
		initializeApplet("lowlevel")

		// initialize NAND flash applet
		initializeApplet("nandflash")

		// erase all memory
		applet.erase(0, applet.memorySize)

		// write files
		applet.write(0x000000, "sama5d4-nandflashboot-uboot-4.0.12-rc2.bin", true)
		applet.write(0x040000, "irdm.bin")
	}
}
