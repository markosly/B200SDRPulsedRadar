#include "wfm.h"
double wfm::sampPower(int idx)
{
	double result = samples[idx].real()*samples[idx].real()+samples[idx].imag()*samples[idx].imag();
	return result;

}

wfm wfm::PwrXcor(wfm refWfm,int numSampsNeg, int numSampsPos)
{
	wfm result;
	if (refWfm.Fs != Fs) return result; // wont work if sample rates dont match
	result.samples.resize(numSampsNeg+numSampsPos+1);
	result.Fs = refWfm.Fs;
	pow(10,5);
	int index;

	for(int offset = -numSampsNeg; offset < numSampsPos;offset++)
	{
		for(int i = 0; i < samples.size();i++)
		{
			index = (offset+i+samples.size())%samples.size();
			result.samples[offset+numSampsNeg] = result.samples[offset+numSampsNeg] + complex<float>(sampPower(index)*refWfm.sampPower(i),0);
		}

	}
return result;
}
double wfm::findMaxPowerIdx(int lowestIdx, int highestIdx)
{
	int peakIdx=lowestIdx;
	double peakVal=0.0;
	for(int i = lowestIdx; i <= highestIdx;i++)
	{
		if(sampPower(i)> peakVal)
		{
			peakVal = sampPower(i);
			peakIdx = i;
		}

	}

	// now use parabolic fit to interpolate where the acualt peak idx is
	// fit the parabola to the peakIdx and 2 adjacent indices
	int leftIdx;
	int rightIdx;
	double leftVal;
	double rightVal;
	leftIdx = peakIdx -1;
	rightIdx = peakIdx+1;
	leftVal = sampPower(leftIdx);
	rightVal = sampPower(rightIdx);
/*
	denom = (x1 - x2)(x1 - x3)(x2 - x3)
	A = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
	B = (x3^2 * (y1 - y2) + x2^2 * (y3 - y1) + x1^2 * (y2 - y3)) / denom
	C = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom
	*/
	double denom = (leftIdx - peakIdx)*(leftIdx - rightIdx)*(peakIdx - rightIdx);
	double A = (rightIdx * (peakVal - leftVal) + peakIdx * (leftVal - rightVal) + leftIdx * (rightVal - peakVal)) / denom;
	double B = (rightIdx*rightIdx * (leftVal - peakVal) + peakIdx*peakIdx * (rightVal - leftVal) + leftIdx*leftIdx * (peakVal - rightVal)) / denom;
	double C = (peakIdx * rightIdx * (peakIdx - rightIdx) * leftVal + rightIdx * leftIdx * (rightIdx - leftIdx) * peakVal + leftIdx * peakIdx * (leftIdx - peakIdx) * rightVal) / denom;
	double result = -B / (2*A);
	return result;
}
void wfm::FFT(void)
{
fftw_complex *in,*out;
fftw_plan p;
in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*samples.size());
out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*samples.size());
double powerPrior = calcWfmPower();
for(int i = 0; i < samples.size();i++)
{
	in[i][0]= samples[i].real();
	in[i][1]=samples[i].imag();
}
p = fftw_plan_dft_1d(samples.size(),in,out,FFTW_FORWARD,FFTW_ESTIMATE);
fftw_execute(p);

fftw_destroy_plan(p);
for(int i = 0; i < samples.size();i++)
	{
		samples[i]= complex<float>(out[i][0],out[i][1]);
	}
setWfmPower(powerPrior);
fftw_free(in);
fftw_free(out);

}
void wfm::IFFT(void)
{
fftw_complex *in,*out;
fftw_plan p;
in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*samples.size());
out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*samples.size());
double powerPrior = calcWfmPower();
for(int i = 0; i < samples.size();i++)
{
	in[i][0]= samples[i].real();
	in[i][1]=samples[i].imag();
}
p = fftw_plan_dft_1d(samples.size(),in,out,FFTW_BACKWARD,FFTW_ESTIMATE);
fftw_execute(p);
for(int i = 0; i < samples.size();i++)
	{
		samples[i]= complex<float>(out[i][0],out[i][1]);
	}
fftw_destroy_plan(p);
setWfmPower(powerPrior);
fftw_free(in);
fftw_free(out);

}
double wfm::calcWfmPower(void)
{
	double res=0;
	for(int i = 0; i < samples.size();i++)res = res + sampPower(i);
	res = res/samples.size();
	return res;

}

void wfm::setWfmPower(double power)
{
double currentPower = calcWfmPower();
double powerRatio = power/currentPower;
double voltageRatio = pow(powerRatio,0.5);
for(int i = 0; i < samples.size();i++)samples[i] = samples[i]*complex<float>(voltageRatio,0);
return;
}

void wfm::resample(int upsample,int decimate)
{
	wfm workingWfm;
	workingWfm.samples.resize(samples.size()*upsample);
	workingWfm.Fs = Fs*upsample;
	double finalFs = (Fs*upsample)/decimate;
	// first upsample, then filter then decimate
	if(upsample > 1)
	{
		for(int i = 0; i < samples.size()*upsample;i++)
		{
			if((i%upsample)==0) workingWfm.samples[i] = samples[i/upsample];
			else workingWfm.samples[i] = complex<float>(0.0,0.0);
		}

	}
	else workingWfm.samples = samples;
	// upsampling(interleaving 0s done
	workingWfm.FFT();
	if((Fs/2)<(finalFs/2))workingWfm.fftLowPassFilter(Fs/2); //interpolation filter/ decimation, done in one step
	else workingWfm.fftLowPassFilter(finalFs/2);
	//std::cout << "In resample, cutoff Freq: "<< finalFs/2<<endl;
	workingWfm.IFFT();
	// now decimate
	samples.resize(workingWfm.samples.size()/decimate);

	for(int i = 0; i < samples.size();i++)
	{
		samples[i] = workingWfm.samples[i*decimate];
	}



	Fs = finalFs;
return;

}
void wfm::fftLowPassFilter(double BW)
{
	double binSpacing = Fs/samples.size();// 1/TimeDuration, where time duration is samples.size()/Fs
	int startIndex;
	int stopIndex;
	//if(BW >= Fs) throw("wfm::fftLowPassFilter:BW exceeds samplerate of signal!");

	startIndex = BW /binSpacing;
	stopIndex = 2*((Fs/2)-BW)/binSpacing + startIndex; // in FFT format bins go from 0 to most pos freq, then most neg freq to least neg
	if(samples.size()%2==0) stopIndex = stopIndex - 1;  // for even length fft, most pos freq and most neg freq share same bin
	for(int i = startIndex; i <= stopIndex;i++)
	{
		samples[i]= complex<float>(0.001,0.0)*samples[i];
	}
	return;
}
void wfm::generatePulseTrain(int pulseLengthSamps, int periodLengthSamps,int numPeriods)
{
	for(int i = 0; i < periodLengthSamps*numPeriods;i++)
	{
		if((i%periodLengthSamps)<pulseLengthSamps) samples[i]= complex<float>(1.0,1.0);
		else samples[i]= complex<float>(0.0,0.0);
	}
	return;
}
void wfm::saveENV(const char *filename)
{
	FILE *fp;


	fp = fopen(filename,"wt");

	if (fp == NULL) return;

	fprintf(fp,"#First line : sample rate (Hz). Data : I (volt), Q (volt)\r\n");




	fprintf(fp,"%lf\n",Fs);
 for (int i = 0; i < samples.size(); i++) {
		fprintf(fp,"%f,%f\n",samples[i].real(),samples[i].imag());
	}

	fclose(fp);

	return;
}
void wfm::scaleMaxIQ(double ampl)
{
	double peakI = 0.0;
	double peakQ = 0.0;
	double scaleFactor = 1.0;
	for(int i = 0; i < samples.size();i++)
	{
		if(samples[i].real()>peakI)peakI = samples[i].real();
		if(samples[i].imag()>peakQ)peakQ = samples[i].imag();
	}
	if(peakI > peakQ) scaleFactor = ampl/peakI;
	else scaleFactor = ampl/peakQ;
	for(int i = 0; i < samples.size();i++)samples[i] = samples[i]* complex<float>(scaleFactor,0.0);
	return;
}
int wfm::size(void)
{
	return samples.size();
}
complex<float> & wfm::operator[](int idx)
{
	return samples[idx];
}
void wfm::resize(int size)
{
	samples.resize(size);
}
void wfm::clearIQ(void)
{
	for(int i = 0; i < size(); i++) samples[i]= complex<float>(0.0,0.0);
return;
}
double wfm::findHighestPowerPeak(int lowestIdx, int highestIdx)
{
	if(lowestIdx < 1) lowestIdx = 1;
	if(highestIdx > (size()-2)) highestIdx = size()-2;
	int peakIdx=lowestIdx;
	double peakVal=0.0;
	for(int i = lowestIdx; i <= highestIdx;i++)
	{
		if((sampPower(i)> peakVal)&& (sampPower(i)>sampPower(i-1))&&(sampPower(i)>sampPower(i+1)))
		{
			peakVal = sampPower(i);
			peakIdx = i;
		}

	}

	// now use parabolic fit to interpolate where the acualt peak idx is
	// fit the parabola to the peakIdx and 2 adjacent indices
	int leftIdx;
	int rightIdx;
	double leftVal;
	double rightVal;
	leftIdx = peakIdx -1;
	rightIdx = peakIdx+1;
	leftVal = sampPower(leftIdx);
	rightVal = sampPower(rightIdx);
/*
	denom = (x1 - x2)(x1 - x3)(x2 - x3)
	A = (x3 * (y2 - y1) + x2 * (y1 - y3) + x1 * (y3 - y2)) / denom
	B = (x3^2 * (y1 - y2) + x2^2 * (y3 - y1) + x1^2 * (y2 - y3)) / denom
	C = (x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3) / denom
	*/
	double denom = (leftIdx - peakIdx)*(leftIdx - rightIdx)*(peakIdx - rightIdx);
	double A = (rightIdx * (peakVal - leftVal) + peakIdx * (leftVal - rightVal) + leftIdx * (rightVal - peakVal)) / denom;
	double B = (rightIdx*rightIdx * (leftVal - peakVal) + peakIdx*peakIdx * (rightVal - leftVal) + leftIdx*leftIdx * (peakVal - rightVal)) / denom;
	double C = (peakIdx * rightIdx * (peakIdx - rightIdx) * leftVal + rightIdx * leftIdx * (rightIdx - leftIdx) * peakVal + leftIdx * peakIdx * (leftIdx - peakIdx) * rightVal) / denom;
	double result = -B / (2*A);
	return result;

}
