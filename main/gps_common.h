/**
 * 	@file	gps_common.h
 * 	@brief 	Common structure and functions related to GPS module
 *	@author	Visakhan
 *	@date	Jun 27, 2015
 */

#include <stdint.h>
#include <stdbool.h>


#ifndef GPS_COMMON_H
#define GPS_COMMON_H

/**
 * @brief Structure to hold time from GPS
 */
typedef struct _gps_time
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

} gps_time_struct, *gps_time_struct_ptr;

/**
 * @brief Possible fix types for GPS
 */
typedef enum _gps_fix
{
	GPS_NOFIX = 0, //!< No Fix
	GPS_FIX = 1,//!< GPS Fix
	DGPS_FIX = 2//!< DGPS Fix
} gps_fix_type;


/**
 * @brief 	Structure containing various GPS parameters. A global variable
 * 			of this structure type holds current GPS status, updated periodically
 */
typedef struct _gps_info
{
	/* Time */
	gps_time_struct time;	/**< Date and time  */

	/* Position */
	//volatile char noth_south;		/**< North-South indicator for latitude: 'N' or 'S' */
	//volatile char east_west;			/**< East-West indicator for longitude: 'E' or 'W' */
	volatile float latitude;			/**< Latitude with 6 digit fractional part */
	volatile float longitude;		/**< Longitude with 6 digit fractional part */
	volatile float hdop;				/**< Horizontal degree of precision */
	volatile int velocity;			/**< Velocity in kmph */
	volatile int course;				/**< Course (direction) in degrees */
	volatile int altitude;			/**< MSL Altitude (meters) */
	volatile gps_fix_type fix;		/**< Fix type */
	volatile int sat_used;			/**< Number of used satellites */
	volatile bool ext_antenna;		/**< External Antenna connected? */

} gps_info_struct, *gps_info_struct_ptr;


extern gps_info_struct gps_info;

/**
 * @brief Parse a GGA sentence in NMEA format and updates GPS info structure
 * @param buf Buffer containing sentence received from GPS
 * @param gps_info_ptr GPS info structure to update with parsed values
 * @return Returns 0 always
 */
extern int gps_parse_gga(const char *buf, gps_info_struct_ptr gps_info_ptr);


/**
 * @brief Parse a RMC sentence in NMEA format and updates GPS info structure
 * @param buf Buffer containing sentence received from GPS
 * @param gps_info_ptr GPS info structure to update with parsed values
 * @return Returns 0 always
 */
extern int gps_parse_rmc(const char *buf, gps_info_struct_ptr gps_info_ptr);


/**
 * @brief Converts GPS time (GMT format) to IST time format
 * @param gps_time Structure containing GPS format time (See @ref gps_time_struct)
 * @param ist_time Structure to store IST time
 */
extern void gps_time_to_ist(gps_time_struct *gps_time, gps_time_struct *ist_time);


/**
 * 	@brief 	Send PMTK command to GPS module through UART
 * 	@param	cmdBuf : Buffer containing command string in the format: "$PMTK...*"
 * 					 (buffer should have 4 extra bytes left for appending checksum and CR-LF)
 * 	@param	len	: Number of characters in the command string
 *  @retval Command success status
 * 			0 - Success
 * 			1 - GPS UART Timeout
 * 			2 - GPS ACK Error
 */
extern uint32_t gps_send_cmd(uint8_t *cmdBuf, uint32_t len);



/**
 * 	@brief Function to send given data to GPS module through UART
 * 	@param buf : Buffer containing the text to send
 * 	@param len : Number of characters to send
 * 	@retval Result of transmit operation
 * 			0 - Success
 * 			1 - UART Tx Error
 */
extern uint32_t gps_send(uint8_t *buf, uint32_t len);

#endif /* SOURCES_GPS_PARSE_H_ */
