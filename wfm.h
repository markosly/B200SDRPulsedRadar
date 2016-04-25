/*Written by Marko Slijepcevic Jan 2016 for the purpose of implementing an SDR based Radar system
 *
 */
#include <complex>
#include <fftw3.h>
#include <vector>
#include <iostream>
using namespace std;
class wfm
{
public:

	vector<complex<float> > samples; // time domain IQ samples

	double Fs; // sampleRate
	// This function shifts the local wfm over the offsets specified and generates the resulting xcor for those offsets
	// refWfm should be longer than or same in length as the local wfm
	//negative shift represents delaying the local relative to the ref wfm, positive shift represents delaying the ref relative to the local
	wfm PwrXcor(wfm refWfm,int numSampsNeg, int numSampsPos); //
	void resample(int upsample,int decimate);
	// returns idx of peak power point, note this is an interpolated double precision value
	//lastly, this function will not work if the data contains 3 or more consecutive peak value points
	double findMaxPowerIdx(int lowestIdx, int highestIdx);
	double sampPower(int idx);
	void FFT(void);
	void IFFT(void);
	double calcWfmPower(void);
	void setWfmPower(double power);
	// attenuates all frequencies outside of +/- BW by 60dB
	void fftLowPassFilter(double BW);
	//pulseLengthSamps is num samps with power
	// periodLengthSamps is time between rising edges of pulses ie each period is this many samples long
	//numperiods is obvious
	void generatePulseTrain(int pulseLengthSamps, int periodLengthSamps,int numPeriods);
	void saveENV(const char *filename);
	void scaleMaxIQ(double ampl); // rescales the IQ such that the max value in either the I or Q channel is the value given by ampl
	int size(void);
	complex<float> & operator[](int idx);
	void resize(int size);
	void clearIQ(void); // overwrites all IQ data with 0.0,0.0

	// finds the highest local maximum in the range specified
	//note, the lowest legal value for startIdx is 1 and the highest legal value for stopIdx is size()-2
	//lastly, this function will not work if the data contains 3 or more consecutive peak value points
	double findHighestPowerPeak(int lowestIdx, int highestIdx);
};
