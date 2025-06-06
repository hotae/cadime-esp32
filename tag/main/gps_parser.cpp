#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "gps_parser.h"
#include "math.h"

#define INDENT_SPACES 

struct minmea_sentence_rmc frame;

int gps_parser(const char* line)
{
    int result = 0;
    ///struct minmea_sentence_rmc frame;
    //char line[MINMEA_MAX_SENTENCE_LENGTH];
    //while (fgets(line, sizeof(line), stdin) != NULL) {
    if (line != NULL)
    {
        ///sprintf(line, "$GNRMC,082403.00,A,3724.41359,N,12705.32317,E,0.488,,210524,,,A,V*10");
        ///printf("%s", line);
        switch (minmea_sentence_id(line, false)) {
            case MINMEA_SENTENCE_RMC: {
                if (minmea_parse_rmc(&frame, line)) {
                    // printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%ld/%ld,%ld/%ld) %ld/%ld\n",
                    //         frame.latitude.value, frame.latitude.scale,
                    //         frame.longitude.value, frame.longitude.scale,
                    //         frame.speed.value, frame.speed.scale);
                    // printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%ld,%ld) %ld\n",
                    //         minmea_rescale(&frame.latitude, 1000),
                    //         minmea_rescale(&frame.longitude, 1000),
                    //         minmea_rescale(&frame.speed, 1000));
                    // printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                    //         minmea_tocoord(&frame.latitude),
                    //         minmea_tocoord(&frame.longitude),
                    //         minmea_tofloat(&frame.speed));
                    if (isnan(minmea_tocoord(&frame.latitude)) == false && isnan(minmea_tocoord(&frame.longitude)) == false)
                    {
                        result = 1;
                    }
                }
                else {
                    printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_GGA: {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, line)) {
                    printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
                    result = 1;
                }
                else {
                    printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_GST: {
                struct minmea_sentence_gst frame;
                if (minmea_parse_gst(&frame, line)) {
                    printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%ld/%ld,%ld/%ld,%ld/%ld)\n",
                            frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
                            frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
                            frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
                    printf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
                           " scaled to one decimal place: (%ld,%ld,%ld)\n",
                            minmea_rescale(&frame.latitude_error_deviation, 10),
                            minmea_rescale(&frame.longitude_error_deviation, 10),
                            minmea_rescale(&frame.altitude_error_deviation, 10));
                    printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
                            minmea_tofloat(&frame.latitude_error_deviation),
                            minmea_tofloat(&frame.longitude_error_deviation),
                            minmea_tofloat(&frame.altitude_error_deviation));
                }
                else {
                    printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_GSV: {
                struct minmea_sentence_gsv frame;
                if (minmea_parse_gsv(&frame, line)) {
                    printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                    printf(INDENT_SPACES "$xxGSV: satellites in view: %d\n", frame.total_sats);
                    for (int i = 0; i < 4; i++)
                        printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                            frame.sats[i].nr,
                            frame.sats[i].elevation,
                            frame.sats[i].azimuth,
                            frame.sats[i].snr);
                }
                else {
                    printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
                }
            } break;

            case MINMEA_SENTENCE_VTG: {
               struct minmea_sentence_vtg frame;
               if (minmea_parse_vtg(&frame, line)) {
                    printf(INDENT_SPACES "$xxVTG: true track degrees = %f\n",
                           minmea_tofloat(&frame.true_track_degrees));
                    printf(INDENT_SPACES "        magnetic track degrees = %f\n",
                           minmea_tofloat(&frame.magnetic_track_degrees));
                    printf(INDENT_SPACES "        speed knots = %f\n",
                            minmea_tofloat(&frame.speed_knots));
                    printf(INDENT_SPACES "        speed kph = %f\n",
                            minmea_tofloat(&frame.speed_kph));
               }
               else {
                    printf(INDENT_SPACES "$xxVTG sentence is not parsed\n");
               }
            } break;

            case MINMEA_SENTENCE_ZDA: {
                struct minmea_sentence_zda frame;
                if (minmea_parse_zda(&frame, line)) {
                    printf(INDENT_SPACES "$xxZDA: %d:%d:%d %02d.%02d.%d UTC%+03d:%02d\n",
                           frame.time.hours,
                           frame.time.minutes,
                           frame.time.seconds,
                           frame.date.day,
                           frame.date.month,
                           frame.date.year,
                           frame.hour_offset,
                           frame.minute_offset);
                }
                else {
                    printf(INDENT_SPACES "$xxZDA sentence is not parsed\n");
                }
            } break;

            case MINMEA_INVALID: {
                printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
            } break;

            default: {
                printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
            } break;
        }
    }

    return result;
}
