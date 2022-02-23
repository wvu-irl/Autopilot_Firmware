/*
 * Config.h
 *
 *  Created on: May 26, 2015
 *      Author: Matt Gramlich
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "../../Definitions.h"

#include <system.h>
#include <utils.h>
#include <constants.h>
#include <bsp.h>
#include <nettypes.h>

namespace Config
{
	union cfgToByte
	{
		ConfigRecord cfg;
		BYTE result[sizeof(ConfigRecord)];
	}__attribute__ ((packed));

	void updateConfig()
	{
		cfgToByte temp;
		temp.cfg = gConfigRec;
		temp.cfg.ip_Addr = AsciiToIp(localIP);
		temp.cfg.ip_Mask = AsciiToIp(subnetMask);
		temp.cfg.wait_seconds = bootDelay;
		temp.cfg.ser_boot = monitorPort;
		UpdateConfigRecord(&temp.cfg);
		ForceReboot();
	}

	bool checkConfig()
	{
		cfgToByte temp, temp1;
		temp.cfg = gConfigRec;
		temp1.cfg = gConfigRec;
		temp.cfg.ip_Addr = AsciiToIp(localIP);
		temp.cfg.ip_Mask = AsciiToIp(subnetMask);
		temp.cfg.wait_seconds = bootDelay;
		temp.cfg.ser_boot = monitorPort;
		for(unsigned int i = 0; i < sizeof(ConfigRecord); i++)
		{
			if(temp.result[i] != temp1.result[i])
				return false;
		}
		return true;
	}
}


#endif /* CONFIG_H_ */
