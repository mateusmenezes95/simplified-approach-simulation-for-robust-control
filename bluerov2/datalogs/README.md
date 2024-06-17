# Datalogs

## Test data

The test data was gathered from experiments conducted at the facilities of SENAI CIMATEC. We used a standard configuration of the BlueROV2 and the T200 thruster for these tests. The data collection was facilitated by the [Blue Robotics Ardusub](https://www.ardusub.com/) software and [QGroundControl]. The resulting data logs were then converted into CSV format using the [mavlogdump.py] tool.

For example, to generate the CSV file with the AHR2 data, you can use the following command:

```bash
mavlogdump.py --format=csv --csv_sep=';' --types=AHR2 --condition='AHR2.TimeUS>=16331019632 and RCOU.TimeUS<=16809520107' 00-log-with-useful-data.bin > bluerov2/datalogs/tests-on_04-23-2024/analysis-interval-01/ahr2-data.csv
```

See the [Ardusub Logging documentation] for more information on how to convert the logs to CSV format.

## Thruster T200 data

The file `t200-public-performance-data-10-20v-september-2019.xlsx` was obtained from the [T200 Performance Charts] and contains the data for the T200 thruster.

[Ardusub Logging documentation]: https://www.ardusub.com/reference/data-logging.html
[QGroundControl]: https://www.ardusub.com/reference/qgc-configuration.html
[mavlogdump.py]: https://github.com/ArduPilot/pymavlink/blob/master/tools/mavlogdump.py
[T200 Performance Charts]: https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/ 
