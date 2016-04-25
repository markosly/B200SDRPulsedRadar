//This software is based on the Ettus Research Benchmark rate example provided with the UHD driver.
// it has been heavily modified to facilitate the Radar application.


// Copyright 2011-2013 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <uhd/utils/thread_priority.hpp>
#include <uhd/convert.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <strstream>
#include <fstream>
#include <deque>
#include <complex>
#include <cstdlib>
#include <sys/time.h>
#include <sys/resource.h>

#include "wfm.h"
#include "MTimer.h"
#include <easyBMP/EasyBMP.h>
#include "easyBMP/EasyBMP_BMP.h"	// Added by ClassView
namespace po = boost::program_options;
using namespace std;
/***********************************************************************
 * Test result variables
 **********************************************************************/
unsigned long long num_overflows = 0;
unsigned long long num_underflows = 0;
unsigned long long num_rx_samps = 0;
unsigned long long num_tx_samps = 0;
unsigned long long num_dropped_samps = 0;
unsigned long long num_seq_errors = 0;
//will generate a heatmap for values between 1 and 0
unsigned int generateHeatMapValue(double value)
{
	if(value > 1)value = 1;
	if(value < 0)value = 0;
	//black, blue, green, yellow,orange,red,white
	const float colorMap[7][3]={{0,0,0},{0,0,255},{0,255,0},{255,255,0},{255,165,0},{255,0,0},{255,255,255}};

	value = value*6;// scale to the size of the color map
	int colderColor = floor(value);
	int hotterColor = colderColor +1;
	if(hotterColor > 6)hotterColor = 6;// only happens if value is exactly 1
	unsigned int R = colorMap[colderColor][0]+(value-colderColor)*(colorMap[hotterColor][0]-colorMap[colderColor][0]);// interpolation, red
	unsigned int G = colorMap[colderColor][1]+(value-colderColor)*(colorMap[hotterColor][1]-colorMap[colderColor][1]);// interpolation, green
	unsigned int B = colorMap[colderColor][2]+(value-colderColor)*(colorMap[hotterColor][2]-colorMap[colderColor][2]);// interpolation, blue
unsigned int res = (R << 16) + (G<<8) + B;
return res;
}
void rxFunc(
        uhd::usrp::multi_usrp::sptr usrp,
        const std::string &rx_cpu,
        uhd::rx_streamer::sptr rx_stream,
        double freq,
		double time,
		int numSamps,
		vector<wfm>*result
		//vector<signal> *result
) {
    uhd::set_thread_priority_safe();

    double interval = 10e-3; //10ms capture interval for multicaps
 //   std::vector< complex<float> > buff; // a vector of complex floats
    vector<vector< complex<float> > > buffs; // a vector of units of the above type(multiple buff units)
    buffs.resize(result->size());
    std::vector<complex<float> *> Rxbuff; // a vector of pointers to(arrays of) complex floats
    for(int i = 0; i < result->size(); i++)
	{
    	buffs[i].resize(numSamps);
    	Rxbuff.push_back(&buffs[i].front());
	}
    uhd::rx_metadata_t rxmd;


	uhd::time_spec_t  theTime;
	theTime = usrp->get_time_now();
	uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);

	cmd.num_samps = numSamps;
	cmd.time_spec = time;
	cmd.stream_now = false;//dont stream now, use time spec
	if(time < theTime.get_real_secs())
	{
		std::cout << "timing problem!" << endl;
		std::cout <<"RXTIMESPEC "<< cmd.time_spec.get_real_secs() << endl;
		std::cout <<"USRP time(RxFunc) "<< theTime.get_real_secs()<<endl;
		std::cout <<"USRP Capture Time(RxFunc) "<< time<<endl;
	}
	for(int i = 0; i < result->size();i++)
	{
		rx_stream->issue_stream_cmd(cmd);
		num_rx_samps = rx_stream->recv(Rxbuff[i], numSamps, rxmd,1,false); // supposed to be a blocking call, may need to be separated from tx thread
	//	cout <<"rx @ t= "<< rxmd.time_spec.get_real_secs()<< endl;
		cmd.time_spec = cmd.time_spec.get_real_secs()+interval;
		if(rxmd.error_code != 0)
		{
			std::cout << "rxmd. "<< rxmd.time_spec.get_real_secs()<<std::endl;
			std::cout << "Stored "<< num_rx_samps << " samples."<< std::endl;
			std::cout << "Rx md error code: "<< rxmd.error_code << " RxMD errstr "<< rxmd.strerror()<< std::endl;
		}
	}



	//cout << "Saving data. Vector size: "<< result->size() << endl;
	for(int i = 0; i < result->size();i++)
	{
		(*result)[i].resize(numSamps);

		if(usrp->get_master_clock_rate()>61.44e6)(*result)[i].Fs = 100e6;
		else (*result)[i].Fs = 30e6;


		for(int j = 0; j < numSamps; j ++) (*result)[i][j] = complex<double> (buffs[i][j].real(),buffs[i][j].imag());
		//(*result)[i].sigType = signal::TIMEDOMAIN;

	}

	//(*result)[0].saveENV("//home//guest//Desktop//VM_shared_folder//insideRx.env");
	if(rxmd.error_code != 0)
	{
		std::cout << "rxmd. "<< rxmd.time_spec.get_real_secs()<<std::endl;
		std::cout << "Stored "<< num_rx_samps << " samples."<< std::endl;
		std::cout << "Rx md error code: "<< rxmd.error_code << " RxMD errstr "<< rxmd.strerror()<< std::endl;}
	return;
}
void txFunc(
        uhd::usrp::multi_usrp::sptr usrp,
        const std::string &tx_cpu,
        uhd::tx_streamer::sptr tx_stream,
		double freq,
		double time,
		int numSamps,
		wfm tx1,
		wfm tx2,
		//signal tx1,
		//signal tx2,
		int reps
) {
    uhd::set_thread_priority_safe();
    std::vector< complex<float> > buff(numSamps);
    std::vector< complex<float> > buff2(numSamps);
    // going to start with 10mS repetition rate, may make it variable in future
    double interval = 10e-3;

    for(int i = 0; i < numSamps; i++){
    	buff[i] = complex<float> (tx1[i].real(),tx1[i].imag());
    	buff2 [i]= complex<float> (tx2[i].real(),tx2[i].imag());
    	}

    	std::vector<const void *> Txbuffs;

    	Txbuffs.push_back(&buff.front());
    	Txbuffs.push_back(&buff2.front());

    uhd::tx_metadata_t md;

    md.time_spec = time;
    md.has_time_spec = true;
    uhd::time_spec_t  theTime;
    theTime = usrp->get_time_now();

    uhd::async_metadata_t asmd;
    for( int i = 0; i < reps; i++)
    {
    	md.start_of_burst = true;
    	md.end_of_burst = true;

    	num_tx_samps = tx_stream->send(Txbuffs, numSamps, md,0.5)*tx_stream->get_num_channels();


    	while(usrp->get_time_now().get_real_secs() < (md.time_spec.get_real_secs()+(tx1.size()/tx1.Fs)))boost::this_thread::sleep(boost::posix_time::microseconds(500));


    	md.time_spec = md.time_spec.get_real_secs() + interval;
     }


    //send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send(Txbuffs, 0, md,1);
}



/***********************************************************************
 * Main code + dispatcher
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){


	wfm txWfm;
	txWfm.Fs = 100e6;
	double timeDuration = 1e-3;
	txWfm.samples.resize(txWfm.Fs *timeDuration);
	txWfm.generatePulseTrain(5,500,txWfm.samples.size()/500);// 50nS pulse duration, 5uS pulse period, 1mS total length

	uhd::set_thread_priority_safe();
    setpriority(PRIO_PROCESS, 0, -20);
    ofstream *targetData;
    ofstream *targetVelocity;
    ofstream *log;
    log = new ofstream;
    targetData = new ofstream;
    targetVelocity = new ofstream;
    log->open("//home//guest//Radarlog.txt");
    targetData->open("//home//guest//targetData.txt");
    targetVelocity->open("//home//guest//targetVelocity.txt");
#define lf if(log)(*log)
    lf << "LogfileTest" <<endl;

    //variables to be set by po
    std::string args;
    double duration;
    double rx_rate, tx_rate;
    std::string rx_otw, tx_otw;
    std::string rx_cpu, tx_cpu;
    std::string mode;
    std::string channel_list;


    double TxLOFreq = 5.5e9;
    double RxLOFreq = 5.5e9;

    int numSamps = 1e5;

    double T0 = 0.37;
    vector<wfm> singlePulsetrain;
    singlePulsetrain.resize(1);
   // vector<signal>singleCap;
    vector<wfm>singleCap;
    singleCap.resize(1);
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("duration", po::value<double>(&duration)->default_value(5.0), "duration for the test in seconds")
        ("rx_rate", po::value<double>(&rx_rate), "specify to perform a RX rate test (sps)")
        ("tx_rate", po::value<double>(&tx_rate), "specify to perform a TX rate test (sps)")
        ("rx_otw", po::value<std::string>(&rx_otw)->default_value("sc16"), "specify the over-the-wire sample mode for RX")
        ("tx_otw", po::value<std::string>(&tx_otw)->default_value("sc16"), "specify the over-the-wire sample mode for TX")
        ("rx_cpu", po::value<std::string>(&rx_cpu)->default_value("fc32"), "specify the host/cpu sample mode for RX")
        ("tx_cpu", po::value<std::string>(&tx_cpu)->default_value("fc32"), "specify the host/cpu sample mode for TX")
        ("mode", po::value<std::string>(&mode)->default_value("none"), "multi-channel sync mode option: none, mimo")
        ("random", "Run with random values of samples in send() and recv() to stress-test the I/O.")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help") or (vm.count("rx_rate") + vm.count("tx_rate")) == 0){
        std::cout << boost::format("UHD Benchmark Rate %s") % desc << std::endl;
        std::cout <<
        "    Specify --rx_rate for a receive-only test.\n"
        "    Specify --tx_rate for a transmit-only test.\n"
        "    Specify both options for a full-duplex test.\n"
        << std::endl;
        return ~0;
    }




    //create a usrp device
    std::cout << std::endl;
    uhd::device_addrs_t device_addrs = uhd::device::find(args, uhd::device::USRP);
    if (not device_addrs.empty() and device_addrs.at(0).get("type", "") == "usrp1"){
        std::cerr << "*** Warning! ***" << std::endl;
        std::cerr << "Benchmark results will be inaccurate on USRP1 due to insufficient features.\n" << std::endl;
    }
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;


    boost::thread_group thread_group;

    //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

//begin MYCODE
    mTimer tickTock;

    tickTock.Start();


	usrp->set_time_now(uhd::time_spec_t(0.0)); // set t = 0
	usrp->set_command_time(T0-0.200);
	usrp->set_rx_freq(RxLOFreq);
	usrp->set_tx_freq(TxLOFreq,0);
	usrp->clear_command_time();


	usrp->set_rx_rate(rx_rate);
	usrp->set_rx_gain(74);
	usrp->set_tx_gain(87);

	//create a receive streamer
	uhd::stream_args_t Rxstream_args(rx_cpu, rx_otw);
	vector<size_t>rxChan;
	rxChan.push_back(channel_nums[0]);
	Rxstream_args.channels = rxChan;

	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(Rxstream_args);
	int imageDepth = 10; // this is the total number of passes displayed in the radar image
	BMP radarDiagram;
	wfm dummyWfm = txWfm.PwrXcor(txWfm,-52,145); // delay the reference waveform from 520nS to 1450nS
	radarDiagram.SetSize(10*dummyWfm.size()+10,10*imageDepth+10,0);
	int radarDiagramWidth = 10*dummyWfm.size()+10;
	int radarDiagramHeight = 10*imageDepth+10;



    wfm origWfm = txWfm;
    if(usrp->get_master_clock_rate() < 61.44e6)	txWfm.resample(3,10);

    wfm zerosWfm = txWfm;

    zerosWfm.clearIQ();

    txWfm.scaleMaxIQ(0.3);// to this to avoid overranging the DAC
    lf << "txSig numSamps after resample: "<<numSamps<<endl;
    lf << "txWfm numSamps after resample: "<< txWfm.size()<<endl;
    numSamps = txWfm.size();

    txWfm.saveENV("//home//guest//Desktop//VM_shared_folder//transmittedWfm.env");

    singleCap[0] = txWfm;
    cout << "singleCap[0] sample rate: "<< singleCap[0].Fs << endl;
    ifstream runCheck;
    runCheck.open("//srv//httpd//htdocs//running");
    ifstream killCheck;
    killCheck.open("//srv//httpd//htdocs//killProc");

    deque<wfm>imgCaptures;
    imgCaptures.resize(10);

    for(int i =0 ; i < imgCaptures.size();i++){imgCaptures[i] = zerosWfm;imgCaptures[i].resize(dummyWfm.size());}
    if(runCheck.good())cout << "All systems go."<< std::endl;
    double previousDistance = 1.0;
    double currentDistance = 1.0;
    //double currentSpeed = 0.0;
    double secondPeak = 0.0;
    double firstPeak = 0.0;
    double timeBetweenCaps = 0.0;
    double velocity = 0.0;

while(!killCheck.good())
{
while(runCheck.good())
{

	usrp->set_tx_rate(tx_rate);
	//create a transmit streamer
	uhd::stream_args_t Txstream_args(tx_cpu, tx_otw);
	Txstream_args.channels = channel_nums;
	cout << "about to get stream args"<< endl;
	uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(Txstream_args);
	cout << "about to reset usrp time"<< endl;
	usrp->set_time_now(uhd::time_spec_t(0.0)); // set t = 0
	cout<<"clocking timer"<< endl;
	tickTock.Stop();
	timeBetweenCaps = tickTock.GetDuration();
	lf << "Time since last run: "<< timeBetweenCaps <<endl;
	tickTock.Start();
	tickTock.Stop();
	lf << "Spawning RX Thread"<< endl;
	lf << "T = "<<tickTock.GetDuration()<<endl;

	thread_group.create_thread(boost::bind(&rxFunc, usrp, rx_cpu, rx_stream,RxLOFreq,T0,numSamps,&singleCap));
	tickTock.Stop();
	lf << "spawning Tx thread" << endl;
	thread_group.create_thread(boost::bind(&txFunc, usrp, tx_cpu, tx_stream, TxLOFreq,T0, numSamps, txWfm,zerosWfm,1));


	thread_group.join_all();
	tickTock.Stop();

	lf<< "TX&RX threads done."<< endl;
	lf << "T = "<<tickTock.GetDuration()<<endl;
	timeBetweenCaps += tickTock.GetDuration();
	singleCap[0].saveENV("//home//guest//Desktop//VM_shared_folder//BSeriesTestCapture.env"); // save rx cap to determine rx offset

	singleCap[0].resample(10,3);// peak search is done at 100msps for compatibility with better hardware
	singleCap[0].saveENV("//home//guest//Desktop//VM_shared_folder//BSeriesTestCaptureUpsampled.env"); // save rx cap to determine rx offset

    imgCaptures.push_back(singleCap[0].PwrXcor(origWfm,-52,145));

    imgCaptures.back().resize((radarDiagramWidth/10)-1);
    imgCaptures.back().scaleMaxIQ(.99);
	lf << "xcored"<<endl;
   // imageCaptures.pop_front();
	imgCaptures.pop_front();
    lf<<"popped back"<<endl;
    lf<<"imagesCaptures.size()="<< imgCaptures.size() << endl;
    lf<<"imagesCaptures[0].size()="<< imgCaptures[0].size() << endl;
    lf<<"maxwidth to be accessed: "<< 9+10*(imgCaptures[0].size()-1)+1<<endl;
    lf<<"maxheight to be accessed: "<< 9+10*(imgCaptures.size()-1)+1<<endl;
    for(int j = 0; j < imgCaptures.size();j++) // green bars
    {

    for(int i = 0; i < imgCaptures[0].size();i++)
    {

    	//int truncatedVal = fabs(imgCaptures[j][i].real()*65536);
    	//truncatedVal = truncatedVal & 0xFF00;
    	int truncatedVal = generateHeatMapValue(imgCaptures[j][i].real());
    	//for(int k = 0 ; k < 10 ; k++) for(int l = 0; l <10 ; l++) radarDiagram.SetPixel(l+10*i+1,k+10*j+1,truncatedVal);
    	for(int k = 0 ; k < 10 ; k++) for(int l = 0; l <10 ; l++) radarDiagram.SetPixel(l+10*i+1,k+10*j+1,generateHeatMapValue(imgCaptures[j][i].real()));
    }
    }

    for(int q = 0; q <radarDiagramWidth; q = q + 50)// blue ticks
    {
    	if(q+6 > radarDiagramWidth)break;
    	for(int k = 0; k < 3; k++)for(int j = 0; j < 10; j++) radarDiagram.SetPixel(q+k+3,radarDiagramHeight-10+j,255);

    }
    lf<< "checking if still active"<<endl;
	runCheck.close();
	runCheck.open("//srv//httpd//htdocs//running");
	int minRange = 3;
	lf << "Min Range Setting(idx): "<< minRange <<endl;
	// erase old red tick
	for(int k = 0; k < 3; k++)for(int j = 0; j < 10; j++) radarDiagram.SetPixel(secondPeak*10+k+3,radarDiagramHeight-10+j,0x000000);

	lf <<"Peak Detection wfm Fs: "<< imgCaptures.back().Fs<<endl;
	imgCaptures.back().saveENV("//home//guest//Desktop//VM_shared_folder//imgcapsback.env");
	firstPeak = imgCaptures.back().findMaxPowerIdx(0,minRange);

	secondPeak = imgCaptures.back().findHighestPowerPeak(minRange+firstPeak,imgCaptures.back().size()-1);
	lf << "firstpeak: "<< firstPeak << endl;
	lf << "secondPeak: "<< secondPeak << endl;



	//new red ticks
	for(int k = 0; k < 3; k++)for(int j = 0; j < 10; j++) radarDiagram.SetPixel(secondPeak*10+k+3,radarDiagramHeight-10+j,0xFF0000);

	currentDistance = (secondPeak - firstPeak)*3/2;
	velocity = (currentDistance-previousDistance)/(timeBetweenCaps/1000);// delta x over delta t (t is in milliseconds)
	lf << "uncalibrated distance(m): "<< currentDistance <<endl;
	lf << "velocity(m/s): "<< velocity << endl;
	(*targetData)<< currentDistance << endl;
	(*targetVelocity)<< velocity << endl;
	previousDistance = currentDistance;
	radarDiagram.WriteToFile("//srv//httpd//htdocs//test.bmp");

	std::cout << "Diagram Written" << endl;
	cout<< "Rx Power: "<< singleCap[0].calcWfmPower()<<endl;
	}
killCheck.close();
killCheck.open("//srv//httpd//htdocs//stop");
if(!killCheck.good()) boost::this_thread::sleep(boost::posix_time::microseconds(100e3)); // wait
   }
unlink("//srv//httpd//htdocs//stop"); // for deleting
    //print summary
    std::cout << std::endl << boost::format(
        "Benchmark rate summary:\n"
        "  Num received samples:    %u\n"
        "  Num dropped samples:     %u\n"
        "  Num overflows detected:  %u\n"
        "  Num transmitted samples: %u\n"
        "  Num sequence errors:     %u\n"
        "  Num underflows detected: %u\n"
    ) % num_rx_samps % num_dropped_samps % num_overflows % num_tx_samps % num_seq_errors % num_underflows << std::endl;

    //finished
    //testSig->saveENV("//home//guest//result.env");


    log->close();
    targetData->close();
    delete(log);
    delete (targetData);
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    return EXIT_SUCCESS;
}
