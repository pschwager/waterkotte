#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

static uint16_t CRC16_BUYPASS(uint8_t *data, size_t len) {
	uint16_t crc = 0x0000;
	size_t j;
	int i;
	for (j=len; j>0; j--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (i=0; i<8; i++) {
			if (crc & 0x8000) crc = (crc<<1) ^ 0x8005;
			else crc <<= 1;
		}
	}
	return (crc);
}

int main(void) {
	char *portname = "/dev/serial/by-id/usb-067b_2303-if00-port0";
	int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "error %d opening %s: %s", errno, portname, strerror(errno));
		return -1;
	}
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		fprintf(stderr, "error %d from tcgetattr", errno);
		return -1;
	}
	cfsetispeed(&tty,B9600);
	cfsetospeed(&tty,B9600);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	 // 8-bit chars
	tty.c_cflag &= ~(PARENB | PARODD);			// shut off parity
	tty.c_cflag &= ~CSTOPB;						// CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	tty.c_cflag &= ~CRTSCTS;					// No Hardware flow Control
	tty.c_cflag |= (CLOCAL | CREAD);			// ignore modem controls, enable reading

	// disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
	tty.c_iflag = 0;
	tty.c_iflag &= ~IGNBRK;						// disable break processing
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);		// shut off xon/xoff ctrl
	tty.c_iflag &= ~ICANON;						// Non Cannonical mode

	tty.c_oflag = 0;							// no remapping, no delays

	tty.c_lflag = 0;							// no signaling chars, no echo, no canonical processing

//	tty.c_cc[VMIN] = 255;						// read doesn't block
	tty.c_cc[VMIN] = 0;						// read doesn't block
	tty.c_cc[VTIME] = 20;						// 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "error %d from tcsetattr", errno);
		return -1;
	}
	tcflush(fd, TCIFLUSH);	// Discards old data in the rx buffer
	char write_at_buffer[] = {0x41, 0x54, 0x0D};
	if(write(fd,write_at_buffer,sizeof(write_at_buffer)) < 0) {
		fprintf(stderr, "error writing AT", errno);
	}
//	for(int try=0; try<10; try++) {
		usleep(100000);
		tcflush(fd, TCIFLUSH);	// Discards old data in the rx buffer
		char write_cmd_buffer[] = {0x10, 0x02, 0x01, 0x15, 0x00, 0x00, 0x01, 0x52, 0x10, 0x03, 0x79, 0xF4};
		if(write(fd,write_cmd_buffer,sizeof(write_cmd_buffer)) < 0) {
			fprintf(stderr, "error writing command", errno);
		}
		int i = 0;
		char last = 0;
		char read_buffer[400];
		char result_buffer[352];
		int resultCounter = 0;
		int bytes_read = 0;
		for(;read(fd, &read_buffer[bytes_read], 1) != 0; bytes_read++);
	/*	fprintf(stderr, "Bytes Rxed: %d\n", bytes_read);
		for(i=0;i<bytes_read;i++)
			fprintf(stderr, "%3.2x",read_buffer[i]);
		fprintf(stderr, "\n");
	*/
		if(bytes_read==0) {
			return 0;
		}
		if(read_buffer[0]!=22 || read_buffer[1]!=16 || read_buffer[2]!=2 || read_buffer[3]!=0 || read_buffer[4]!=23) {
			fprintf(stderr, "start bits wrong!\n");
			for(i=0;i<bytes_read;i++)
				fprintf(stderr, "%3.2x",read_buffer[i]);
			fprintf(stderr, "\n\n");
			for(i=0;i<resultCounter;i++) {
				fprintf(stderr, "%3.2x",result_buffer[i]);
			}
			fprintf(stderr, "\n\n");
			//return -1;
		} else {
			if(read_buffer[bytes_read-5]!=16 || read_buffer[bytes_read-4]!=3 || read_buffer[bytes_read-1]!=22) {
				fprintf(stderr, "end bits wrong!\n");
				for(i=0;i<bytes_read;i++)
					fprintf(stderr, "%3.2x",read_buffer[i]);
				fprintf(stderr, "\n\n");
				for(i=0;i<resultCounter;i++) {
					fprintf(stderr, "%3.2x",result_buffer[i]);
				}
				fprintf(stderr, "\n\n");
				//return -1;
			} else {
				for(i=5;i<bytes_read-5;i++) {
					if(read_buffer[i]==16) { // Skip duplicate hex 0x10
						i++;
					}
					result_buffer[resultCounter++]=read_buffer[i];
				}
			/*	fprintf(stderr, "\n\n");
				for(i=0;i<resultCounter;i++) {
					fprintf(stderr, "%3.2x",result_buffer[i]);
				}
				fprintf(stderr, "\n\n");
			*/	uint16_t calcCRC=CRC16_BUYPASS(&read_buffer[3], bytes_read-8);
				uint16_t readCRC = read_buffer[bytes_read-2] | (uint16_t)read_buffer[bytes_read-3] << 8;
				if(calcCRC != readCRC) {
					fprintf(stderr, "CRC mismatch: calculated 0x%04x, read 0x%04x\n", calcCRC, readCRC);
					for(i=0;i<bytes_read;i++)
						fprintf(stderr, "%3.2x",read_buffer[i]);
					fprintf(stderr, "\n\n");
					for(i=0;i<resultCounter;i++) {
						fprintf(stderr, "%3.2x",result_buffer[i]);
					}
					fprintf(stderr, "\n\n");
					//return -1;
					usleep(1000000);
				} else {
					short s;
					float f;
					memcpy(&f, &result_buffer[79], 4);
					if(f == 0.0f) {
						fprintf(stderr, "Kompressorbetriebsstunden == 0, invalid data");
						//return -1;
						usleep(1000000);
					} else {
						if(result_buffer[88] > 0) {
							for(i=0;i<bytes_read;i++)
								fprintf(stderr, "%3.2x",read_buffer[i]);
							fprintf(stderr, "\n\n");
							for(i=0;i<resultCounter;i++) {
								fprintf(stderr, "%3.2x",result_buffer[i]);
							}
							fprintf(stderr, "\n\n");




						memcpy(&f, &result_buffer[0], 4);
						fprintf(stderr, "Temp-Aussen=%f\n",f);
						memcpy(&f, &result_buffer[4], 4);
						fprintf(stderr, "Temp-Aussen-1h=%f\n", f);
						memcpy(&f, &result_buffer[8], 4);
						fprintf(stderr, "Temp-Aussen-24h=%f\n", f);
						memcpy(&f, &result_buffer[12], 4);
						fprintf(stderr, "Temp-Ruecklauf-Soll=%f\n", f);
						memcpy(&f, &result_buffer[16], 4);
						fprintf(stderr, "Temp-Ruecklauf=%f\n", f);
						memcpy(&f, &result_buffer[20], 4);
						fprintf(stderr, "Temp-Vorlauf=%f\n", f);
						memcpy(&f, &result_buffer[24], 4);
						fprintf(stderr, "Temp-Raum=%f\n", f);
						memcpy(&f, &result_buffer[28], 4);
						fprintf(stderr, "Temp-Raum-1h=%f\n", f);
						memcpy(&f, &result_buffer[32], 4);
						fprintf(stderr, "Temp-WQuelle-Ein=%f\n", f);
						memcpy(&f, &result_buffer[36], 4);
						fprintf(stderr, "Temp-WQuelle-Aus=%f\n", f);
						memcpy(&f, &result_buffer[40], 4);
						fprintf(stderr, "Temp-Verdampfer=%f\n", f);
						memcpy(&f, &result_buffer[44], 4);
						fprintf(stderr, "Temp-Kondensator=%f\n", f);
						memcpy(&f, &result_buffer[48], 4);
						fprintf(stderr, "Ww-Temp=%f\n", f);
						fprintf(stderr, "Uhrzeit=%02d:%02d:%02d\n", result_buffer[54], result_buffer[53], result_buffer[52]);
						fprintf(stderr, "Datum=%02d.%02d.%02d\n", result_buffer[55], result_buffer[56], result_buffer[57]);
						fprintf(stderr, "Messbeginn-Zeit=%02d:%02d:%02d\n", result_buffer[60], result_buffer[59], result_buffer[58]);
						fprintf(stderr, "Messbeginn-Datum=%02d.%02d.%02d\n", result_buffer[61], result_buffer[62], result_buffer[63]);
						memcpy(&f, &result_buffer[64], 4);
						fprintf(stderr, "Hz-Messergebnis=%f\n", f);
						memcpy(&f, &result_buffer[68], 4);
						fprintf(stderr, "Ww-Messergebnis=%f\n", f);
						fprintf(stderr, "Mess-Reset=%d\n", result_buffer[72]);
						fprintf(stderr, "KomprBeginn-Zeit=%02d:%02d:%02d\n", result_buffer[75], result_buffer[74], result_buffer[73]);
						fprintf(stderr, "KomprBeginn-Datum=%02d.%02d.%02d\n", result_buffer[76], result_buffer[77], result_buffer[78]);
						memcpy(&f, &result_buffer[79], 4);
						fprintf(stderr, "KomprBetrStunden=%f\n", f);
						fprintf(stderr, "Kompr-Mess-Reset=%d\n", result_buffer[83]);
						if(result_buffer[84] > 0)
							fprintf(stderr, "Unterbrechungen=%d\n", result_buffer[84]);
						if(result_buffer[85] > 0)
							fprintf(stderr, "Warnung-Eingang=%d\n", result_buffer[85]);
						if(result_buffer[86] > 0)
							fprintf(stderr, "Warnung-Ausgang=%d\n", result_buffer[86]);
						if(result_buffer[87] > 0)
							fprintf(stderr, "Warnung-Sonstige=%d\n", result_buffer[87]);
						if(result_buffer[88] > 0)
							fprintf(stderr, "Ausfaelle=%d\n", result_buffer[88]);
						if(result_buffer[89] != 128)
							fprintf(stderr, "Fuehler-Ausfall=%d\n", result_buffer[89]);
						if(result_buffer[89] & (1<<0))
							fprintf(stderr, "- Fuehler-Ausfall 0 Außentemparatur\n");
						if(result_buffer[89] & (1<<1))
							fprintf(stderr, "- Fuehler-Ausfall 1 Quelle Eingang\n");
						if(result_buffer[89] & (1<<2))
							fprintf(stderr, "- Fuehler-Ausfall 2 Quelle Ausgang\n");
						if(result_buffer[89] & (1<<3))
							fprintf(stderr, "- Fuehler-Ausfall 3 Verdampfer\n");
						if(result_buffer[89] & (1<<4))
							fprintf(stderr, "- Fuehler-Ausfall 4 Rücklauf\n");
						if(result_buffer[89] & (1<<5))
							fprintf(stderr, "- Fuehler-Ausfall 5 Vorlauf\n");
						if(result_buffer[89] & (1<<6))
							fprintf(stderr, "- Fuehler-Ausfall 6 Kondensator\n");
						if(result_buffer[89] & (1<<7))
							fprintf(stderr, "- Fuehler-Ausfall 7 Warmwasser\n");
						if(result_buffer[90] > 0)
							fprintf(stderr, "Fuehler-KurzSchl=%d\n", result_buffer[90]);
						memcpy(&s, &result_buffer[91], 2);
						fprintf(stderr, "Kontrollwert für Fühlerkalibrierung (FuehlerZaehler0)=%d\n", s);
						fprintf(stderr, "FuehlRaum-Ausfall=%d\n", result_buffer[93]);
						fprintf(stderr, "FuehlRaum-KurzSchl=%d\n", result_buffer[94]);
						memcpy(&s, &result_buffer[95], 2);
						fprintf(stderr, "FuehlRaum-Zaehler0=%d\n", s);
						fprintf(stderr, "Ausfall-Zeit=%02d:%02d:%02d\n", result_buffer[99], result_buffer[98], result_buffer[97]);
						fprintf(stderr, "Ausfall-Datum=%02d.%02d.%02d\n", result_buffer[100], result_buffer[101], result_buffer[102]);
						fprintf(stderr, "Ausfall-Betriebszust.=%d\n", result_buffer[103]);
						fprintf(stderr, "Ausfall-Do-Buffer=%d\n", result_buffer[104]);
						fprintf(stderr, "Ausfall-Di-Buffer=%d\n", result_buffer[105]);
						fprintf(stderr, "Ausfall-FuehlAusfall=%d\n", result_buffer[106]);
						fprintf(stderr, "Ausfall-FuehlKurzSchl=%d\n", result_buffer[107]);
						memcpy(&f, &result_buffer[108], 4);
						fprintf(stderr, "Ausfall-Temp-Aussen=%f\n", f);
						memcpy(&f, &result_buffer[112], 4);
						fprintf(stderr, "Ausfall-Temp-WQuelle-Ein=%f\n", f);
						memcpy(&f, &result_buffer[116], 4);
						fprintf(stderr, "Ausfall-Temp-WQuelle-Aus=%f\n", f);
						memcpy(&f, &result_buffer[120], 4);
						fprintf(stderr, "Ausfall-Temp-Verdampfer=%f\n", f);
						memcpy(&f, &result_buffer[124], 4);
						fprintf(stderr, "Ausfall-Temp-Ruecklauf=%f\n", f);
						memcpy(&f, &result_buffer[128], 4);
						fprintf(stderr, "Ausfall-Temp-Vorlauf=%f\n", f);
						memcpy(&f, &result_buffer[132], 4);
						fprintf(stderr, "Ausfall-Temp-Kondensator=%f\n", f);
						memcpy(&f, &result_buffer[136], 4);
						fprintf(stderr, "Ausfall-Temp-Ww=%f\n", f);
						fprintf(stderr, "Ausfall-RaumAusfall=%d\n", result_buffer[140]);
						fprintf(stderr, "Ausfall-RaumKurzsch=%d\n", result_buffer[141]);
						memcpy(&f, &result_buffer[142], 4);
						fprintf(stderr, "Ausfall-Temp-Raum=%f\n", f);
						fprintf(stderr, "Ausfall-Reset=%d\n", result_buffer[146]);
						fprintf(stderr, "Kennwort=%d\n", result_buffer[147]);
						fprintf(stderr, "Werkseinstellung=%d\n", result_buffer[148]);
						fprintf(stderr, "ResetAnforderung=%d\n", result_buffer[149]);
						fprintf(stderr, "Betriebszustaende=%d\n", result_buffer[150]);
						if(result_buffer[150] & (1<<0))
							fprintf(stderr, "- Betriebszustaende 0 Warmwasser Aktiv\n");
						if(result_buffer[150] & (1<<1))
							fprintf(stderr, "- Betriebszustaende 1 Heizung Aktiv\n");
						if(result_buffer[150] & (1<<2))
							fprintf(stderr, "- Betriebszustaende 2 Stufe 2 Aktiv\n");
						if(result_buffer[150] & (1<<3))
							fprintf(stderr, "- Betriebszustaende 3 Hand-Betrieb Aktiv\n");
						if(result_buffer[150] & (1<<4))
							fprintf(stderr, "- Betriebszustaende 4 Unterbrechung\n");
						if(result_buffer[150] & (1<<5))
							fprintf(stderr, "- Betriebszustaende 5 TempDiff HzgAus-HzgEin > OK \n");
						if(result_buffer[150] & (1<<6))
							fprintf(stderr, "- Betriebszustaende 6 -\n");
						if(result_buffer[150] & (1<<7))
							fprintf(stderr, "- Betriebszustaende 7 Externe Steuerung\n");
						fprintf(stderr, "Do-Buffer Relaisausgänge=%d\n", result_buffer[151]);
						if(result_buffer[151] & (1<<0))
							fprintf(stderr, "- Do-Buffer 0 Magnetventil\n");
						if(result_buffer[151] & (1<<1))
							fprintf(stderr, "- Do-Buffer 1 Kompressor\n");
						if(result_buffer[151] & (1<<2))
							fprintf(stderr, "- Do-Buffer 2 Alarm Sammelstörmeldung\n");
						if(result_buffer[151] & (1<<3))
							fprintf(stderr, "- Do-Buffer 3 Umschaltventil Kühlung (Kurbelwannenheizung)\n");
						if(result_buffer[151] & (1<<4))
							fprintf(stderr, "- Do-Buffer 4 Stufe 2\n");
						if(result_buffer[151] & (1<<5))
							fprintf(stderr, "- Do-Buffer 5 Pumpe Heizung\n");
						if(result_buffer[151] & (1<<6))
							fprintf(stderr, "- Do-Buffer 6 Pumpe Warmwasser / 3-Wege Brauchwasser\n");
						if(result_buffer[151] & (1<<7))
							fprintf(stderr, "- Do-Buffer 7 Pumpe Quelle\n");
						fprintf(stderr, "Di-Buffer Digitaleingänge=%d\n", result_buffer[152]);
						if(result_buffer[152] & (1<<0))
							fprintf(stderr, "- Di-Buffer 0 Ausfall Öldruck-Pressostat / Motorschutzkompressor\n");
						if(result_buffer[152] & (1<<1))
							fprintf(stderr, "- Di-Buffer 1 Ausfall Niederdruck Pressostat\n");
						if(result_buffer[152] & (1<<2))
							fprintf(stderr, "- Di-Buffer 2 Ausfall Hochdruck Pressostat\n");
						if(result_buffer[152] & (1<<3))
							fprintf(stderr, "- Di-Buffer 3 Ausfall Pumpe Quelle\n");
						if(result_buffer[152] & (1<<4))
							fprintf(stderr, "- Di-Buffer 4 Meldung Systemdruck Sole Minimum\n");
						if(result_buffer[152] & (1<<5))
							fprintf(stderr, "- Di-Buffer 5 frei\n");
						if(result_buffer[152] & (1<<6))
							fprintf(stderr, "- Di-Buffer 6 Externe Sollwertbeeinflussung\n");
						if(result_buffer[152] & (1<<7))
							fprintf(stderr, "- Di-Buffer 7 Externe Abschaltung\n");
						memcpy(&s, &result_buffer[153], 2);
						fprintf(stderr, "Status-Gesamt=%d\n", s);
						memcpy(&s, &result_buffer[155], 2);
						fprintf(stderr, "Status-Verriegel=%d\n", s);
						memcpy(&s, &result_buffer[157], 2);
						fprintf(stderr, "Status-Heizung=%d\n", s);
						memcpy(&s, &result_buffer[159], 2);
						fprintf(stderr, "Status-Stufe2=%d\n", s);
						memcpy(&s, &result_buffer[161], 2);
						fprintf(stderr, "Status-Wasser=%d\n", s);
						memcpy(&s, &result_buffer[164], 2);
						fprintf(stderr, "Status-WPumpe=%d\n", s);
						fprintf(stderr, "Mode-Heizung=%d\n", result_buffer[165]);
						if(result_buffer[165] & (1<<0))
							fprintf(stderr, "- Mode-Heizung 0 Normaler Heizvorgang\n");
						if(result_buffer[165] & (1<<1))
							fprintf(stderr, "- Mode-Heizung 1 Externe Sollwertanhebung aktiv\n");
						if(result_buffer[165] & (1<<2))
							fprintf(stderr, "- Mode-Heizung 2 Schnellaufheizung aktiv\n");
						if(result_buffer[165] & (1<<3))
							fprintf(stderr, "- Mode-Heizung 3 Sommer\n");
						if(result_buffer[165] & (1<<4))
							fprintf(stderr, "- Mode-Heizung 4 Zeitprogramm\n");
						if(result_buffer[165] & (1<<5))
							fprintf(stderr, "- Mode-Heizung 5 Unterdrückt\n");
						if(result_buffer[165] & (1<<6))
							fprintf(stderr, "- Mode-Heizung 6 Kein Bedarf\n");
						if(result_buffer[165] & (1<<7))
							fprintf(stderr, "- Mode-Heizung 7 Unterbrechung Fühlerfehler\n");
						fprintf(stderr, "Mode-Wasser=%d\n", result_buffer[166]);
						fprintf(stderr, "Versions-Datum=%02d.%02d.%02d\n", result_buffer[167], result_buffer[168], result_buffer[169]);
						fprintf(stderr, "CPU-Boot-Zeit=%02d:%02d:%02d\n", result_buffer[172], result_buffer[171], result_buffer[170]);
						fprintf(stderr, "CPU-Boot-Datum=%02d.%02d.%02d\n", result_buffer[173], result_buffer[174], result_buffer[175]);
						memcpy(&s, &result_buffer[176], 2);
						fprintf(stderr, "CRC-Summe=%x\n", s);
						fprintf(stderr, "Neu-Start=%d\n", result_buffer[178]);
						fprintf(stderr, "Hz-Abschaltung=%d\n", result_buffer[179]);
						memcpy(&f, &result_buffer[180], 4);
						fprintf(stderr, "Hz-Temp-Einsatz=%f\n", f);
						memcpy(&f, &result_buffer[184], 4);
						fprintf(stderr, "Hz-Temp-BasisSoll=%f\n", f);
						memcpy(&f, &result_buffer[188], 4);
						fprintf(stderr, "Hz-KlSteilheit=%f\n", f);
						fprintf(stderr, "Hz-SchnellAufhz=%d\n", result_buffer[192]);
						fprintf(stderr, "Hz-Zeit-Ein=%02d:%02d:%02d\n", result_buffer[195], result_buffer[194], result_buffer[193]);
						fprintf(stderr, "Hz-Zeit-Aus=%02d:%02d:%02d\n", result_buffer[198], result_buffer[197], result_buffer[196]);
						fprintf(stderr, "Hz-Anhebung-Ein=%02d:%02d:%02d\n", result_buffer[201], result_buffer[200], result_buffer[199]);
						fprintf(stderr, "Hz-Anhebung-Aus=%02d:%02d:%02d\n", result_buffer[204], result_buffer[203], result_buffer[202]);
						memcpy(&f, &result_buffer[205], 4);
						fprintf(stderr, "Hz-Temp-RaumSoll=%f\n", f);
						fprintf(stderr, "Hz-RaumEinfluss=%d\n", result_buffer[209]);
						memcpy(&f, &result_buffer[210], 4);
						fprintf(stderr, "Hz-Ext-Anhebung=%f\n", f);
						memcpy(&f, &result_buffer[214], 4);
						fprintf(stderr, "Hz-Begrenzung=%f\n", f);
						memcpy(&f, &result_buffer[218], 4);
						fprintf(stderr, "Hz-St2-Begrenzung=%f\n", f);
						memcpy(&f, &result_buffer[222], 4);
						fprintf(stderr, "Hz-Hysterese=%f\n", f);
						fprintf(stderr, "Hz-PumpenNachl=%d\n", result_buffer[226]);
						fprintf(stderr, "Ww-Abschaltung=%d\n", result_buffer[227]);
						fprintf(stderr, "Ww-Zeit-Ein=%02d:%02d:%02d\n", result_buffer[230], result_buffer[229], result_buffer[228]);
						fprintf(stderr, "Ww-Zeit-Aus=%02d:%02d:%02d\n", result_buffer[233], result_buffer[232], result_buffer[231]);
						memcpy(&f, &result_buffer[234], 4);
						fprintf(stderr, "Ww-Temp-Soll=%f\n", f);
						memcpy(&f, &result_buffer[238], 4);
						fprintf(stderr, "Ww-Becken-Temp-Soll=%f\n", f);
						memcpy(&f, &result_buffer[242], 4);
						fprintf(stderr, "Ww-Hysterese=%f\n", f);
						memcpy(&f, &result_buffer[246], 4);
						fprintf(stderr, "Ww-Becken-Hysterese=%f\n", f);
						fprintf(stderr, "Unterdr-Warnung-Eingang=%d\n", result_buffer[250]);
						fprintf(stderr, "Unterdr-Warnung-Ausgang=%d\n", result_buffer[251]);
						fprintf(stderr, "Unterdr-Warnung-Sonstige=%d\n", result_buffer[252]);
						fprintf(stderr, "Betriebs-Mode=%d.%d.%d\n", result_buffer[253], result_buffer[254], result_buffer[255]);
						fprintf(stderr, "Modem-Klingelzeichen=%d\n", result_buffer[256]);
						fprintf(stderr, "Fremdzugriff=%d\n", result_buffer[257]);
						fprintf(stderr, "Schluesselnummer=%d\n", result_buffer[258]);
						fprintf(stderr, "Hz-Ext-Freigabe=%d\n", result_buffer[259]);
						memcpy(&f, &result_buffer[260], 4);
						fprintf(stderr, "Hz-Ext-TempRueckl-Soll=%f\n", f);
						memcpy(&f, &result_buffer[264], 4);
						fprintf(stderr, "Temp-QAus-Min=%f\n", f);
						memcpy(&f, &result_buffer[268], 4);
						fprintf(stderr, "Temp-Verdampfer-Min=%f\n", f);
						fprintf(stderr, "Estrich-Aufhz=%d\n", result_buffer[272]);
						fprintf(stderr, "Hz-Ext-Steuerung=%d\n", result_buffer[273]);
						fprintf(stderr, "St2-bei-EvuAbsch=%d\n", result_buffer[274]);
						fprintf(stderr, "Freigabe-Beckenwasser=%d\n", result_buffer[275]);
						fprintf(stderr, "Do-Handkanal=%d\n", result_buffer[276]);
						fprintf(stderr, "Do-Handkanal-Ein=%d\n", result_buffer[277]);
						memcpy(&f, &result_buffer[278], 4);
						fprintf(stderr, "Analog-Korr-Faktor=%f\n", f);
						fprintf(stderr, "Run-Flag=%d\n", result_buffer[282]);
						fprintf(stderr, "\n\npv ",f);







						} else {
					/*	memcpy(&f, &result_buffer[0], 4);
						fprintf(stderr, "Temp-Aussen=%f\n",f);
						memcpy(&f, &result_buffer[4], 4);
						fprintf(stderr, "Temp-Aussen-1h=%f\n", f);
						memcpy(&f, &result_buffer[8], 4);
						fprintf(stderr, "Temp-Aussen-24h=%f\n", f);
						memcpy(&f, &result_buffer[12], 4);
						fprintf(stderr, "Temp-Ruecklauf-Soll=%f\n", f);
						memcpy(&f, &result_buffer[16], 4);
						fprintf(stderr, "Temp-Ruecklauf=%f\n", f);
						memcpy(&f, &result_buffer[20], 4);
						fprintf(stderr, "Temp-Vorlauf=%f\n", f);
						memcpy(&f, &result_buffer[24], 4);
						fprintf(stderr, "Temp-Raum=%f\n", f);
						memcpy(&f, &result_buffer[28], 4);
						fprintf(stderr, "Temp-Raum-1h=%f\n", f);
						memcpy(&f, &result_buffer[32], 4);
						fprintf(stderr, "Temp-WQuelle-Ein=%f\n", f);
						memcpy(&f, &result_buffer[36], 4);
						fprintf(stderr, "Temp-WQuelle-Aus=%f\n", f);
						memcpy(&f, &result_buffer[40], 4);
						fprintf(stderr, "Temp-Verdampfer=%f\n", f);
						memcpy(&f, &result_buffer[44], 4);
						fprintf(stderr, "Temp-Kondensator=%f\n", f);
						memcpy(&f, &result_buffer[48], 4);
						fprintf(stderr, "Ww-Temp=%f\n", f);
						fprintf(stderr, "Uhrzeit=%02d:%02d:%02d\n", result_buffer[54], result_buffer[53], result_buffer[52]);
						fprintf(stderr, "Datum=%02d.%02d.%02d\n", result_buffer[55], result_buffer[56], result_buffer[57]);
						fprintf(stderr, "Messbeginn-Zeit=%02d:%02d:%02d\n", result_buffer[60], result_buffer[59], result_buffer[58]);
						fprintf(stderr, "Messbeginn-Datum=%02d.%02d.%02d\n", result_buffer[61], result_buffer[62], result_buffer[63]);
						memcpy(&f, &result_buffer[64], 4);
						fprintf(stderr, "Hz-Messergebnis=%f\n", f);
						memcpy(&f, &result_buffer[68], 4);
						fprintf(stderr, "Ww-Messergebnis=%f\n", f);
						fprintf(stderr, "Mess-Reset=%d\n", result_buffer[72]);
						fprintf(stderr, "KomprBeginn-Zeit=%02d:%02d:%02d\n", result_buffer[75], result_buffer[74], result_buffer[73]);
						fprintf(stderr, "KomprBeginn-Datum=%02d.%02d.%02d\n", result_buffer[76], result_buffer[77], result_buffer[78]);
						memcpy(&f, &result_buffer[79], 4);
						fprintf(stderr, "KomprBetrStunden=%f\n", f);
						fprintf(stderr, "Kompr-Mess-Reset=%d\n", result_buffer[83]);
					*/	if(result_buffer[84] > 0)
							fprintf(stderr, "Unterbrechungen=%d\n", result_buffer[84]);
						if(result_buffer[85] > 0)
							fprintf(stderr, "Warnung-Eingang=%d\n", result_buffer[85]);
						if(result_buffer[86] > 0)
							fprintf(stderr, "Warnung-Ausgang=%d\n", result_buffer[86]);
						if(result_buffer[87] > 0)
							fprintf(stderr, "Warnung-Sonstige=%d\n", result_buffer[87]);
						if(result_buffer[88] > 0)
							fprintf(stderr, "Ausfaelle=%d\n", result_buffer[88]);
						if(result_buffer[89] != 128)
							fprintf(stderr, "Fuehler-Ausfall=%d\n", result_buffer[89]);
						if(result_buffer[89] & (1<<0))
							fprintf(stderr, "- Fuehler-Ausfall 0 Außentemparatur\n");
						if(result_buffer[89] & (1<<1))
							fprintf(stderr, "- Fuehler-Ausfall 1 Quelle Eingang\n");
						if(result_buffer[89] & (1<<2))
							fprintf(stderr, "- Fuehler-Ausfall 2 Quelle Ausgang\n");
						if(result_buffer[89] & (1<<3))
							fprintf(stderr, "- Fuehler-Ausfall 3 Verdampfer\n");
						if(result_buffer[89] & (1<<4))
							fprintf(stderr, "- Fuehler-Ausfall 4 Rücklauf\n");
						if(result_buffer[89] & (1<<5))
							fprintf(stderr, "- Fuehler-Ausfall 5 Vorlauf\n");
						if(result_buffer[89] & (1<<6))
							fprintf(stderr, "- Fuehler-Ausfall 6 Kondensator\n");
					/*	if(result_buffer[89] & (1<<7))
							fprintf(stderr, "- Fuehler-Ausfall 7 Warmwasser\n");
					*/
						if(result_buffer[90] > 0)
							fprintf(stderr, "Fuehler-KurzSchl=%d\n", result_buffer[90]);
					/*	memcpy(&s, &result_buffer[91], 2);
						fprintf(stderr, "Kontrollwert für Fühlerkalibrierung (FuehlerZaehler0)=%d\n", s);
						fprintf(stderr, "FuehlRaum-Ausfall=%d\n", result_buffer[93]);
						fprintf(stderr, "FuehlRaum-KurzSchl=%d\n", result_buffer[94]);
						memcpy(&s, &result_buffer[95], 2);
						fprintf(stderr, "FuehlRaum-Zaehler0=%d\n", s);
						fprintf(stderr, "Ausfall-Zeit=%02d:%02d:%02d\n", result_buffer[99], result_buffer[98], result_buffer[97]);
						fprintf(stderr, "Ausfall-Datum=%02d.%02d.%02d\n", result_buffer[100], result_buffer[101], result_buffer[102]);
						fprintf(stderr, "Ausfall-Betriebszust.=%d\n", result_buffer[103]);
						fprintf(stderr, "Ausfall-Do-Buffer=%d\n", result_buffer[104]);
						fprintf(stderr, "Ausfall-Di-Buffer=%d\n", result_buffer[105]);
						fprintf(stderr, "Ausfall-FuehlAusfall=%d\n", result_buffer[106]);
						fprintf(stderr, "Ausfall-FuehlKurzSchl=%d\n", result_buffer[107]);
						memcpy(&f, &result_buffer[108], 4);
						fprintf(stderr, "Ausfall-Temp-Aussen=%f\n", f);
						memcpy(&f, &result_buffer[112], 4);
						fprintf(stderr, "Ausfall-Temp-WQuelle-Ein=%f\n", f);
						memcpy(&f, &result_buffer[116], 4);
						fprintf(stderr, "Ausfall-Temp-WQuelle-Aus=%f\n", f);
						memcpy(&f, &result_buffer[120], 4);
						fprintf(stderr, "Ausfall-Temp-Verdampfer=%f\n", f);
						memcpy(&f, &result_buffer[124], 4);
						fprintf(stderr, "Ausfall-Temp-Ruecklauf=%f\n", f);
						memcpy(&f, &result_buffer[128], 4);
						fprintf(stderr, "Ausfall-Temp-Vorlauf=%f\n", f);
						memcpy(&f, &result_buffer[132], 4);
						fprintf(stderr, "Ausfall-Temp-Kondensator=%f\n", f);
						memcpy(&f, &result_buffer[136], 4);
						fprintf(stderr, "Ausfall-Temp-Ww=%f\n", f);
						fprintf(stderr, "Ausfall-RaumAusfall=%d\n", result_buffer[140]);
						fprintf(stderr, "Ausfall-RaumKurzsch=%d\n", result_buffer[141]);
						memcpy(&f, &result_buffer[142], 4);
						fprintf(stderr, "Ausfall-Temp-Raum=%f\n", f);
						fprintf(stderr, "Ausfall-Reset=%d\n", result_buffer[146]);
						fprintf(stderr, "Kennwort=%d\n", result_buffer[147]);
						fprintf(stderr, "Werkseinstellung=%d\n", result_buffer[148]);
						fprintf(stderr, "ResetAnforderung=%d\n", result_buffer[149]);
						fprintf(stderr, "Betriebszustaende=%d\n", result_buffer[150]);
					*/	if(result_buffer[150] & (1<<0))
							fprintf(stderr, "- Betriebszustaende 0 Warmwasser Aktiv\n");
					/*	if(result_buffer[150] & (1<<1))
							fprintf(stderr, "- Betriebszustaende 1 Heizung Aktiv\n");
					*/	if(result_buffer[150] & (1<<2))
							fprintf(stderr, "- Betriebszustaende 2 Stufe 2 Aktiv\n");
						if(result_buffer[150] & (1<<3))
							fprintf(stderr, "- Betriebszustaende 3 Hand-Betrieb Aktiv\n");
						if(result_buffer[150] & (1<<4))
							fprintf(stderr, "- Betriebszustaende 4 Unterbrechung\n");
						if(result_buffer[150] & (1<<5))
							fprintf(stderr, "- Betriebszustaende 5 TempDiff HzgAus-HzgEin > OK \n");
						if(result_buffer[150] & (1<<6))
							fprintf(stderr, "- Betriebszustaende 6 -\n");
						if(result_buffer[150] & (1<<7))
							fprintf(stderr, "- Betriebszustaende 7 Externe Steuerung\n");
					/*	fprintf(stderr, "Do-Buffer Relaisausgänge=%d\n", result_buffer[151]);
						if(result_buffer[151] & (1<<0))
							fprintf(stderr, "- Do-Buffer 0 Magnetventil\n");
						if(result_buffer[151] & (1<<1))
							fprintf(stderr, "- Do-Buffer 1 Kompressor\n");
					*/	if(result_buffer[151] & (1<<2))
							fprintf(stderr, "- Do-Buffer 2 Alarm Sammelstörmeldung\n");
					/*	if(result_buffer[151] & (1<<3))
							fprintf(stderr, "- Do-Buffer 3 Umschaltventil Kühlung (Kurbelwannenheizung)\n");
					*/	if(result_buffer[151] & (1<<4))
							fprintf(stderr, "- Do-Buffer 4 Stufe 2\n");
					/*	if(result_buffer[151] & (1<<5))
							fprintf(stderr, "- Do-Buffer 5 Pumpe Heizung\n");
					*/	if(result_buffer[151] & (1<<6))
							fprintf(stderr, "- Do-Buffer 6 Pumpe Warmwasser / 3-Wege Brauchwasser\n");
					/*	if(result_buffer[151] & (1<<7))
							fprintf(stderr, "- Do-Buffer 7 Pumpe Quelle\n");
						fprintf(stderr, "Di-Buffer Digitaleingänge=%d\n", result_buffer[152]);
					*/	if(result_buffer[152] & (1<<0))
							fprintf(stderr, "- Di-Buffer 0 Ausfall Öldruck-Pressostat / Motorschutzkompressor\n");
						if(result_buffer[152] & (1<<1))
							fprintf(stderr, "- Di-Buffer 1 Ausfall Niederdruck Pressostat\n");
						if(result_buffer[152] & (1<<2))
							fprintf(stderr, "- Di-Buffer 2 Ausfall Hochdruck Pressostat\n");
						if(result_buffer[152] & (1<<3))
							fprintf(stderr, "- Di-Buffer 3 Ausfall Pumpe Quelle\n");
						if(result_buffer[152] & (1<<4))
							fprintf(stderr, "- Di-Buffer 4 Meldung Systemdruck Sole Minimum\n");
						if(result_buffer[152] & (1<<5))
							fprintf(stderr, "- Di-Buffer 5 frei\n");
						if(result_buffer[152] & (1<<6))
							fprintf(stderr, "- Di-Buffer 6 Externe Sollwertbeeinflussung\n");
						if(result_buffer[152] & (1<<7))
							fprintf(stderr, "- Di-Buffer 7 Externe Abschaltung\n");
					/*	memcpy(&s, &result_buffer[153], 2);
						fprintf(stderr, "Status-Gesamt=%d\n", s);
						memcpy(&s, &result_buffer[155], 2);
						fprintf(stderr, "Status-Verriegel=%d\n", s);
						memcpy(&s, &result_buffer[157], 2);
						fprintf(stderr, "Status-Heizung=%d\n", s);
						memcpy(&s, &result_buffer[159], 2);
						fprintf(stderr, "Status-Stufe2=%d\n", s);
						memcpy(&s, &result_buffer[161], 2);
						fprintf(stderr, "Status-Wasser=%d\n", s);
						memcpy(&s, &result_buffer[164], 2);
						fprintf(stderr, "Status-WPumpe=%d\n", s);
						fprintf(stderr, "Mode-Heizung=%d\n", result_buffer[165]);
						if(result_buffer[165] & (1<<0))
							fprintf(stderr, "- Mode-Heizung 0 Normaler Heizvorgang\n");
						if(result_buffer[165] & (1<<1))
							fprintf(stderr, "- Mode-Heizung 1 Externe Sollwertanhebung aktiv\n");
					*/	if(result_buffer[165] & (1<<2))
							fprintf(stderr, "- Mode-Heizung 2 Schnellaufheizung aktiv\n");
					/*	if(result_buffer[165] & (1<<3))
							fprintf(stderr, "- Mode-Heizung 3 Sommer\n");
						if(result_buffer[165] & (1<<4))
							fprintf(stderr, "- Mode-Heizung 4 Zeitprogramm\n");
						if(result_buffer[165] & (1<<5))
							fprintf(stderr, "- Mode-Heizung 5 Unterdrückt\n");
						if(result_buffer[165] & (1<<6))
							fprintf(stderr, "- Mode-Heizung 6 Kein Bedarf\n");
					*/	if(result_buffer[165] & (1<<7))
							fprintf(stderr, "- Mode-Heizung 7 Unterbrechung Fühlerfehler\n");
					/*	fprintf(stderr, "Mode-Wasser=%d\n", result_buffer[166]);
						fprintf(stderr, "Versions-Datum=%02d.%02d.%02d\n", result_buffer[167], result_buffer[168], result_buffer[169]);
						fprintf(stderr, "CPU-Boot-Zeit=%02d:%02d:%02d\n", result_buffer[172], result_buffer[171], result_buffer[170]);
						fprintf(stderr, "CPU-Boot-Datum=%02d.%02d.%02d\n", result_buffer[173], result_buffer[174], result_buffer[175]);
						memcpy(&s, &result_buffer[176], 2);
						fprintf(stderr, "CRC-Summe=%x\n", s);
						fprintf(stderr, "Neu-Start=%d\n", result_buffer[178]);
						fprintf(stderr, "Hz-Abschaltung=%d\n", result_buffer[179]);
						memcpy(&f, &result_buffer[180], 4);
						fprintf(stderr, "Hz-Temp-Einsatz=%f\n", f);
						memcpy(&f, &result_buffer[184], 4);
						fprintf(stderr, "Hz-Temp-BasisSoll=%f\n", f);
						memcpy(&f, &result_buffer[188], 4);
						fprintf(stderr, "Hz-KlSteilheit=%f\n", f);
						fprintf(stderr, "Hz-SchnellAufhz=%d\n", result_buffer[192]);
						fprintf(stderr, "Hz-Zeit-Ein=%02d:%02d:%02d\n", result_buffer[195], result_buffer[194], result_buffer[193]);
						fprintf(stderr, "Hz-Zeit-Aus=%02d:%02d:%02d\n", result_buffer[198], result_buffer[197], result_buffer[196]);
						fprintf(stderr, "Hz-Anhebung-Ein=%02d:%02d:%02d\n", result_buffer[201], result_buffer[200], result_buffer[199]);
						fprintf(stderr, "Hz-Anhebung-Aus=%02d:%02d:%02d\n", result_buffer[204], result_buffer[203], result_buffer[202]);
						memcpy(&f, &result_buffer[205], 4);
						fprintf(stderr, "Hz-Temp-RaumSoll=%f\n", f);
						fprintf(stderr, "Hz-RaumEinfluss=%d\n", result_buffer[209]);
						memcpy(&f, &result_buffer[210], 4);
						fprintf(stderr, "Hz-Ext-Anhebung=%f\n", f);
						memcpy(&f, &result_buffer[214], 4);
						fprintf(stderr, "Hz-Begrenzung=%f\n", f);
						memcpy(&f, &result_buffer[218], 4);
						fprintf(stderr, "Hz-St2-Begrenzung=%f\n", f);
						memcpy(&f, &result_buffer[222], 4);
						fprintf(stderr, "Hz-Hysterese=%f\n", f);
						fprintf(stderr, "Hz-PumpenNachl=%d\n", result_buffer[226]);
						fprintf(stderr, "Ww-Abschaltung=%d\n", result_buffer[227]);
						fprintf(stderr, "Ww-Zeit-Ein=%02d:%02d:%02d\n", result_buffer[230], result_buffer[229], result_buffer[228]);
						fprintf(stderr, "Ww-Zeit-Aus=%02d:%02d:%02d\n", result_buffer[233], result_buffer[232], result_buffer[231]);
						memcpy(&f, &result_buffer[234], 4);
						fprintf(stderr, "Ww-Temp-Soll=%f\n", f);
						memcpy(&f, &result_buffer[238], 4);
						fprintf(stderr, "Ww-Becken-Temp-Soll=%f\n", f);
						memcpy(&f, &result_buffer[242], 4);
						fprintf(stderr, "Ww-Hysterese=%f\n", f);
						memcpy(&f, &result_buffer[246], 4);
						fprintf(stderr, "Ww-Becken-Hysterese=%f\n", f);
						fprintf(stderr, "Unterdr-Warnung-Eingang=%d\n", result_buffer[250]);
						fprintf(stderr, "Unterdr-Warnung-Ausgang=%d\n", result_buffer[251]);
						fprintf(stderr, "Unterdr-Warnung-Sonstige=%d\n", result_buffer[252]);
						fprintf(stderr, "Betriebs-Mode=%d.%d.%d\n", result_buffer[253], result_buffer[254], result_buffer[255]);
						fprintf(stderr, "Modem-Klingelzeichen=%d\n", result_buffer[256]);
						fprintf(stderr, "Fremdzugriff=%d\n", result_buffer[257]);
						fprintf(stderr, "Schluesselnummer=%d\n", result_buffer[258]);
						fprintf(stderr, "Hz-Ext-Freigabe=%d\n", result_buffer[259]);
						memcpy(&f, &result_buffer[260], 4);
						fprintf(stderr, "Hz-Ext-TempRueckl-Soll=%f\n", f);
						memcpy(&f, &result_buffer[264], 4);
						fprintf(stderr, "Temp-QAus-Min=%f\n", f);
						memcpy(&f, &result_buffer[268], 4);
						fprintf(stderr, "Temp-Verdampfer-Min=%f\n", f);
						fprintf(stderr, "Estrich-Aufhz=%d\n", result_buffer[272]);
						fprintf(stderr, "Hz-Ext-Steuerung=%d\n", result_buffer[273]);
						fprintf(stderr, "St2-bei-EvuAbsch=%d\n", result_buffer[274]);
						fprintf(stderr, "Freigabe-Beckenwasser=%d\n", result_buffer[275]);
						fprintf(stderr, "Do-Handkanal=%d\n", result_buffer[276]);
						fprintf(stderr, "Do-Handkanal-Ein=%d\n", result_buffer[277]);
						memcpy(&f, &result_buffer[278], 4);
						fprintf(stderr, "Analog-Korr-Faktor=%f\n", f);
						fprintf(stderr, "Run-Flag=%d\n", result_buffer[282]);
						fprintf(stderr, "\n\npv ",f);
					*/
						printf("pv ");
						memcpy(&f, &result_buffer[0], 4);
						printf("WPHZTempAussen=%f,",f);
						memcpy(&f, &result_buffer[12], 4);
						printf("WPHZTempRuecklaufSoll=%f,", f);
						memcpy(&f, &result_buffer[16], 4);
						printf("WPHZTempRuecklauf=%f,", f);
						memcpy(&f, &result_buffer[20], 4);
						printf("WPHZTempVorlauf=%f,", f);
						memcpy(&f, &result_buffer[32], 4);
						printf("WPHZTempWQuelleEin=%f,", f);
						memcpy(&f, &result_buffer[36], 4);
						printf("WPHZTempWQuelleAus=%f,", f);
						memcpy(&f, &result_buffer[40], 4);
						printf("WPHZTempVerdampfer=%f,", f);
						memcpy(&f, &result_buffer[44], 4);
						printf("WPHZTempKondensator=%f,", f);
						memcpy(&f, &result_buffer[79], 4);
						printf("WPHZKomprBetrStunden=%f,", f);
						printf("WPHZDoBufferMagnetventil=%i,", (result_buffer[151] & (1<<0)) > 0);
						printf("WPHZDoBufferKompressor=%i,", (result_buffer[151] & (1<<1)) > 0);
						printf("WPHZDoBufferKurbelwannenheizung=%u,", (result_buffer[151] & (1<<3)) > 0);
						printf("WPHZDoBufferPumpeHeizung=%i,", (result_buffer[151] & (1<<5)) > 0);
						printf("WPHZDoBufferPumpeQuelle=%i,", (result_buffer[151] & (1<<7)) > 0);
						printf("WPHZModeNormalerHeizvorgang=%i,", (result_buffer[165] & (1<<0)) > 0);
						printf("WPHZModeExterneSollwertanhebungAktiv=%i,", (result_buffer[165] & (1<<1)) > 0);
						printf("WPHZModeSommer=%i,", (result_buffer[165] & (1<<3)) > 0);
						printf("WPHZModeZeitprogramm=%i,", (result_buffer[165] & (1<<4)) > 0);
						printf("WPHZModeUnterdrückt=%i,", (result_buffer[165] & (1<<5)) > 0);
						printf("WPHZModeKeinBedarf=%i", (result_buffer[165] & (1<<6)) > 0);
//						try=32000;
						}
					}
				}
			}
		}
//	}
	close(fd);
}
