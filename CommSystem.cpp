/**************************************/
/* Name:  Conolly Burgess	          */
/* Team: 4			                  */
/* Language: C++                      */
/* Program: CommSystem_Sample         */
/**************************************/

#include <iostream>
#include <cstring>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <vector>
#include <math.h>
#include <string>
#include <stdint.h>
#include <cstdlib>
#include <time.h>
#include <unistd.h>


using namespace std;

/*
The funtion of this program is to provide the rover communication system with the internal timing and processes to communicate with the TDRS.
This will be achieved in the following stages:
1. Calculating the orbital period of the TDRS after each pass (will have an intitial calibration) to allow for variation in orbit
2. Intitiating Data Upload to the TDRS
3. Intitiating Data Download to the TDRS
4. Partitioning Data Download to Independent Systems
*/

/*
For simplicity, we will assume that the transciever will recieve and transmit data in matrix format. 
The rover will recieve location instructions and TDRS altitude and upload instrument readings, location data, temperature and power levels.
*/

#define initialAltitude 100000
#define initialPeriod 7430.94
#define PI 3.1415926535
#define LunarRadius 1737400
#define GravitationalConstant (6.022 * pow(10,-11))
#define LunarMass (7.34767 * pow(10,22))

/******************************************************************************/
// Functions

/*******************************************************
 * @brief Double to represent double as type uint64_t to convert to 64-bit binary
 * 
 */

union doubleNum {
    double d;
    uint64_t i;
};

/*******************************************************
 * @brief Converts 64-bit binary to a double under the IEEE 754 standard.
 * 
 * @param s String of 64-bit binary to be converted
 * @return double 64-bit double represented by binary
 */
double binaryToDouble(const std::string& s)
{
    unsigned long long x = 0;
    for (std::string::const_iterator it = s.begin(); it != s.end(); ++it)
    {
        x = (x << 1) + (*it - '0');
    }
    double d;
    memcpy(&d, &x, 8);
    return d;
}

/******************************************************************************/
// Classes/Constructors
class SatelliteData
{
    public:
    SatelliteData ( double altitude = initialAltitude, double orbitalPeriod = initialPeriod );
    void PartitionDownloadData();
    void OrbitalCalculation ();
    void UpdateAltitude ();
    double altitude; // Average altitude
    double orbitalPeriod;
    vector <double> navCount; // Raw navigation data
    vector <double> altCount; // Raw altitude data
};

SatelliteData::SatelliteData(double alt, double orbitalP) 
{
    altitude = alt;
    orbitalPeriod = orbitalP;
}

class RoverData
{  
    public:
    void DownloadData(string inputFileName, string outputFileName); //Download Data from the TDRS
    void CompileData(vector <double> doub); //Converts data from rover into binary-type files for transmission
};

/******************************************************************************/
// Satellite Class Member Functions

/*******************************************************
 * @brief Saves downloaded data from Satellite.
 * 
 */
void SatelliteData::PartitionDownloadData()
{
    // Opens input file
    ifstream inputData;
    inputData.open("translatedData.txt");

    //To demonstrate the type of file (on a very small scale) that the rover would receive from the TDRS, a sample file has been included below
    /****************************************
                    SAMPLE FILE 
          X-Coord Y-coord Z-Coord Altitude
            1.0     2.0     3.0    100000
            30.3    40.2    51.7    99099
            -40     -5      300    100111
    *****************************************/

    // Opens/Creates output files
    ofstream navigationData;
    ofstream altitudeData;
    navigationData.open("navigationData.txt");
    altitudeData.open("altitudeData.txt");

    // Creates local variables to write to file
    double preXCoord;
    double preYCoord;
    double preZCoord;
    double preAltData;

    // Reads data from input file into vector data members until no data is left
    while (inputData >> preXCoord >> preYCoord >> preZCoord >> preAltData)
    {
        // Reads the data into the corresponding output file
        navigationData << preXCoord << " " << preYCoord << " " << preZCoord << " " << endl;
        altitudeData << preAltData << endl;
    
        // Writes data to corresponding vectors 
        navCount.push_back(preXCoord);
        navCount.push_back(preYCoord);
        navCount.push_back(preZCoord);
        altCount.push_back(preAltData);   
    }

    inputData.close();
    navigationData.close();
    altitudeData.close();
}

/*******************************************************
 * @brief Updates orbital period of Satellite based on downloaded values.
 * 
 */
void SatelliteData::OrbitalCalculation()
{
    // Breaks up Kepler's third law to make calculation readable
    double numerator = 4*PI*pow((altitude+LunarRadius),3);
    double denominator = (GravitationalConstant * LunarMass);

    // Calculates the period and assigns it to the orbit period of the Satellite
    double period = pow( ( numerator / denominator ), 1.0/2);
    orbitalPeriod = period;

}

/*******************************************************
 * @brief Updates average altitude of Satellite based on downloaded altitude values.
 * 
 */
void SatelliteData::UpdateAltitude()
{
    // Takes size and initiates altitude sum
    double count = altCount.size();
    double altitudeTotal = 0;

    // Loops through vector to find sum
    for (int j =0; j < count; j++) {
        altitudeTotal += altCount[j];
    }

    // Takes average altitude
    double altitudeAverage = altitudeTotal / count;

    altitude = altitudeAverage;
}

/******************************************************************************/
// RoverData Class Member Functions

/*******************************************************
 * @brief Reads 64-bit binary values from input txt file and inputs double values into output file and vector. Binary values are separated by line.
 * 
 * @param inputFileName Name of input file
 * @param outputFileName Name of output file
 */
void RoverData::DownloadData(string inputFileName, string outputFileName) {
    
    //Like before, a sample file has been provided below for clairity. It is important to note that the diagonositic category is of an undefined length to account for the needs of the rover. 
    //This will allow the rover to communicate more efficiently by not sending full diagnostics every orbital period. 

    /*****************************************************************************************************************************************************
                                                                    SAMPLE FILE 
        Element (Atomic Number)     Percent Composition X-Coord Y-Coord Z-Coord  Internal Temperature(ºC)   External Temperature(ºC) Average Power Use(W) Uptime(s)
                Oxygen              14.1          400    -114.5  568.0           35                        -200                      179.3         40000
                Hydrogen            10.2
                Carbon              75.7            
    
    *******************************************************************************************************************************************************/

    ifstream inputFile;
    ofstream outputFile;
    inputFile.open(inputFileName);
    outputFile.open(outputFileName);

    // Used to store double values from binary input files
    vector<double> data;
    string tempString;

    // Reads converted 64-bit binary data into vector and output file
    while (inputFile >> tempString) {

        outputFile << binaryToDouble(tempString) << endl;

        data.push_back(binaryToDouble(tempString));
    }

    inputFile.close();
    outputFile.close();

}

/*******************************************************
 * @brief Outputs a vector of doubles as 64-bit binary values separated by newlines. Outputs to "package.txt"
 * 
 * @param doub Double vector to be written
 */
void RoverData::CompileData(vector <double> doub) {

    ofstream package;
    package.open("package.txt");

    // Writes doubles to binary values
    for (int j = 0; j < doub.size(); j++) {
        union doubleNum n;
        n.d = doub[j];
        
        int i;
        uint64_t base = 1;
        
        package << (n.i & (base << 63));
        
        for (i = 62; i > 51; --i) {
            package << ((n.i & (base << i)) >> i);
        }
        
        for (; i >= 0; --i) {
            package << ((n.i & (base << i)) >> i);
        }

        // Ends with a newline
        package << endl;

    }

    package.close();
    
}

/******************************************************************************
 * main
 */
int main()
{
    // Prints out initial altitude and period
    cout << "Initial Altitude is: " << initialAltitude << endl;
    cout << "Initial Period is: " << initialPeriod << endl;

    // Instantiates Satellite
    SatelliteData mySatellite;

    // Instantiates rover
    RoverData myRover;

    // Checks if signal is received from the rover, initiating the constant connection
    int signalReceived = 0;

    // Time that the rover has had a constant connection
    int secondsOn = 0;

    while (!signalReceived)
    {
        // If signal is not read, check for signal in file
        ifstream signalConnection;
        signalConnection.open("signalCondition.txt");

        signalConnection >> signalReceived;

        signalConnection.close();

        // Sleeps 1 second until trying again
        sleep(1);


        if (signalReceived) {

            // Downloads data from Satellite and updates values
            mySatellite.PartitionDownloadData();
            mySatellite.UpdateAltitude();
            mySatellite.OrbitalCalculation();

            // Prints values to screen
            cout << "Altitude is: " << mySatellite.altitude << endl;
            cout << "Period is: " << mySatellite.orbitalPeriod << endl;

            // Uploads data from rover to Satellite
            myRover.CompileData(mySatellite.navCount);

            // Downloads and processes data from Satellite
            myRover.DownloadData("InputFile.txt", "output.txt");            
            
            // Records the start time 
            int start = time(NULL);

            // Runs until the end of time
            while (true) {

                float wait = mySatellite.orbitalPeriod;

                // Updates time since startup
                
                secondsOn = time(NULL) - start;
f
                while (abs(secondsOn - mySatellite.orbitalPeriod) < 30) {
                    
                    if (signalReceived)
                    {
                    // Downloads data from Satellite and updates values
                    mySatellite.PartitionDownloadData();
                    mySatellite.UpdateAltitude();
                    mySatellite.OrbitalCalculation();

                    // Prints values to screen to demonstrate the program's effectiveness
                    cout << "Altitude is: " << mySatellite.altitude << endl;
                    cout << "Period is: " << mySatellite.orbitalPeriod << endl;

                    // Uploads data from rover to Satellite
                    myRover.CompileData(mySatellite.navCount);

                    // Downloads and processes data from Satellite
                    myRover.DownloadData("InputFile.txt", "output.txt");  
                    
                    start = time(NULL);
                    }
                }

                sleep(1);
                
            }
        }
    }
}