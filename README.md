WGPU Notes:

-We need to be able to store data on the GPU
-We do that with buffers
-A Vertex buffer will store the data that is common for all types of that shape
-An instance buffer stores the data for each instance
    - for example, vertex buffers might have the given vertex array layout and generalized values for a given shape
    - instance data would then have appropriate scale factors and location and rotation of the vertices for a given object


Time Notes:
see this for a great explanation
https://www.cnmoc.usff.navy.mil/Our-Commands/United-States-Naval-Observatory/Precise-Time-Department/Global-Positioning-System/USNO-GPS-Time-Transfer/Leap-Seconds/

- There are different important time systems in space applications

    - UTC :
        - synced with earth's rotation
        - a day is expected to be exactly 86400 seconds in UTC, but due to accel and decel of the earth due to braking action of tides and orbital perturbations, the current day is actually 86400.002, a value that changes over time
        - 86400.002 over the course of a year is roughly 1 sec, any time that value is more than 0.9 seconds for a year, a leap second is added (or subtracted)
        - UTC is the only time scale that accounts for leap seconds on this list, the rest of the systems are continuous
        - when a leap second is added, the last time on 12/31 of that year becomes 23:59:60.999 instead of 23:59:59.999, thus lagging continuous time by 1 second
        - note that 11:59:60.999 is not a valid time in most programs and must be handled accordingly

    - TAI:
        - continuous time system based on high precision atomic clocks
        - initial epoch Jan 1, 1958, 00:00:00
        - as of 10/14/2024, TAI is ahead of UTC by 37 seconds
    
    - GPS: 
        - continous time system initially synced to UTC 
        - initial epoch of Jan 6, 1980, 00:00:00, thus GPS time is always 19 seconds behind TAI (number of leap seconds between 1/1/1958 and 1/6/1980)
        - GPS time was equal to UTC on the epoch, but additional leap seconds have been added, as of 10/14/2024 GPS is ahead of UTC by 18 sec

    - Ephemeris Time (ET) [old]:
        - The first standard time based on observations of orbital positions used to calculated ephemerides of celestial bodies
        - Replaced in 1984 by TT and TDB to account for relativistic effects on time, which ET did not

    
    - Terrestial Time (TT):  
        - continuous time system based on the proper time on Earth's geoid
        - TAI + 32.184s to be consistent with the old and deprecated Ephemeris Time (ET)

    - Barycentric Dynamical Time (TDB): 
        - relativistic coordinate time scale, intended for astronomical use as a time standard to take account of time dilation
        - is TT + some periodic effects, but the periodic effects will remain less than 2 ms for several millenia
        - for all intents and purposes in nadir, TT and TDB are assumed to be equal, but we will technically use TT

These are all the time systems, but each time system can be expressed in the following formats:

    - DateTime: 10/14/2024 11:04:00.999 for example
    - Julian Date: number of decimal days since noon Jan 1, 4713 BC. J2000 epoch is 2451545.0
    - Ephemeris Time (ET) [new]: The NAIF now uses ephemeris time to mean seconds past j2000. Note that they expect the time to be in the time system TDB, or j2000(TDB)

