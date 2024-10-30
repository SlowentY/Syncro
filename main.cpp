//2023.10.03 duepp program format research
//2023.10.06 +due_download_prog() file output
//2023.10.07  +int due_download_prog_save_to_file, +loop for {events}
//10.18 +int due_download_prog_save_to_file_command/data
//11.01 +serial
//13.11.2023 + test program on real data
//06.02.2023 + reading sync pulse sequence from csv file
//22.05.2024 + added out stream to function due_upload_trajectory in serial_lib_cpp.cpp
//25.05.2024 + added stream ability in serial_lib_cpp.cpp


//digital pin 33 -> RF
//digital pin 40 -> ADC
//digital pin 5 -> RS485 converter direction
//digital pin 6 -> GRU



#include <iostream>

#include<iomanip> // for setbase(), works for base 8,10 and 16 only

#include <thread>
#include <chrono>
#include <ctime>
#include "serial_lib_cpp.cpp"
#include <sstream>
#include <fstream>
#include <list>
#include <iterator>
#include <stdlib.h>
#include <vector>
#include "serialib.h" // Serial library
#include "pugixml.hpp"
#include "simplelogger.hpp"

#define DEBUG_ENABLED

using namespace std;
using namespace std::literals::chrono_literals;

//#define SERIAL_PORT "\\\\.\\COM1"
//const string SERIAL_PORT{"\\\\.\\COM19"};

const string SERIAL_PORT{"\\\\.\\COM5"};
extern std::ostream out(std::cout.rdbuf());
extern SimpleLogger newlogger = SimpleLogger(out, "sync");

static bool en_cout = false;
static bool en_debug = false;

int32_t ii, jj, kk, nn, reps, maxnb;
char ch;
char buffer[255];
char dstr[1024];
uint8_t b[255];

uint32_t outputs1, outputs2, ticks1, ticks2;
uint32_t outputs3, outputs4, ticks3, ticks4;
uint32_t outputs5, outputs6, ticks5, ticks6;
uint32_t outputs7, outputs8, ticks7, ticks8;
uint32_t outputs9, outputs10, ticks9, ticks10;
uint32_t outputs11, ticks11;

/*uint32_t to_uint32_t(string param){

    try{
        uint32_t result = stod(param);
        if (result > 0){
            return uint32_t(result);
    }}//try
    catch (std::invalid_argument const& ex){
        newlogger << "XML parse error std::invalid_argument:: " << ex.what() <<" error in param: "<<param<< '\n';
        return 0;
    }
    catch (std::out_of_range const& ex){
         newlogger << "XML parse error std::out_of_range::" << ex.what() <<" error in param: "<<param<< '\n';
         return 0;
    }

    }//to_uint32_t*/
namespace utilityFunctions {
uint32_t to_uint32_t(std::string param){
            uint32_t result = stoi(param);
            if (result >= 0){
                return uint32_t(result);
            }
            else{
                throw std::invalid_argument("Result isn't integer type\n");
            }
}


bool CheckNode(string param, pugi::xml_node node)
{
    if (node==nullptr)
    {
        newlogger << LogPref::Flag(ERROR) << "Parameter " << param << " not found"<< std::endl;
        return false;
    }
    if (strlen(node.text().get())==0)
    {
        newlogger << LogPref::Flag(ERROR) << "No text for parameter " << param << std::endl;
        return false;
    }

//    if(en_debug)
//    {
//        newlogger << LogPref::Flag(DEBUG) << "Showing parameter : " << param << std::endl;
//        newlogger << LogPref::Flag(DEBUG) << param << " get text: " << node.text().get() << std::endl;
//        newlogger << LogPref::Flag(DEBUG) << param << " get as int : " << to_uint32_t(node.text().get())<< std::endl;
//    }

    //newlogger << param << " get as double : " << node.text().as_double() << std::endl;
    //newlogger << param << " get as bool: " << node.text().as_bool() << std::endl;

    return true;
} //function ShowParam
}

uint32_t GetOutputs(uint32_t RF, uint32_t SW, uint32_t ADC, uint32_t GRU){
    std::stringstream debug_ss;
    uint32_t outputs{0x03000000};
    switch(RF){
    case 0:
        outputs = outputs&0xFFFFFFFE;
        debug_ss << "\nRF 0"<< endl;
        break;
    case 1:
        outputs = outputs|0x00000001;
        debug_ss << "\nRF 1"<< endl;
        break;
    }//end switch RF

    debug_ss << hex << outputs << endl;

    switch(SW){
    case 0:
        outputs = outputs&0xFFFFFFEF;
        debug_ss << "SW 0" << endl;
        break;

    case 1:
        outputs = outputs|0x00000010;
        debug_ss << "SW 1" <<endl;
        break;

    }//end switch SW

    debug_ss << hex << outputs << endl;

    switch(ADC){
    case 0:
        outputs = outputs&0xFFFFFEFF;
        debug_ss << "ADC 0" << endl;
        break;

    case 1:
        outputs = outputs|0x00000100;
        debug_ss << "ADC 1" << endl;
        break;

    }//end switch ADC

    debug_ss << hex << outputs << endl;


    switch(GRU){
    case 0:
        outputs = outputs&0xFCFFFFFF;
        debug_ss << "GRU 0" << endl;
        break;

    case 1:
        outputs = outputs|0x03000000;
        debug_ss << "GRU 1" << endl;
        break;

    }//end switch ADC

    debug_ss << hex << outputs << endl;

   // if(en_debug)
   //     newlogger << LogPref::Flag(DEBUG) << debug_ss.str();

    return outputs;
}

// xml handler
int evalXml(int argc, char ** argv, pugi::xml_document& doc, std::vector<std::vector<uint32_t>>& vec){
        // Handle input xml
        try {
            if (argc < 2) {
                throw std::invalid_argument("Incorrect number of arguments");
            }

            newlogger << "Try to load file...\n";
            pugi::xml_parse_result result = doc.load_file(argv[1]);

            if (!result) {
                 throw std::invalid_argument("Error in loading file");
            }
        }
        catch(const std::invalid_argument& e) {
            newlogger << LogPref::Flag(ERROR) << e.what() << std::endl;
            return 6;
        }

        newlogger << LogPref::Flag(OK) << "External XML structure is OK\n";


        // Check, is this file stucture fits us
        uint32_t ParamCount =  utilityFunctions::to_uint32_t(doc.child("root").child("ParamCount").text().get());

        if(en_debug)
            newlogger << LogPref::Flag(DEBUG) << "ParamCount = " << ParamCount << std::endl;

        // Data arrays (vectors)
        std::vector<uint32_t> RF_array;
        std::vector<uint32_t> SW_array;
        std::vector<uint32_t> ADC_array;
        std::vector<uint32_t> GX_array;
        std::vector<uint32_t> GY_array;
        std::vector<uint32_t> GZ_array;
        std::vector<uint32_t> CL_array;

        try {
            for (size_t i = 1; i <= ParamCount; i++){
                std::string param1 = "RF"+std::to_string(i);
                std::string param2 = "SW"+std::to_string(i);
                std::string param3 = "ADC"+std::to_string(i);
                std::string param4 = "GX"+std::to_string(i);
                std::string param5 = "GY"+std::to_string(i);
                std::string param6 = "GZ"+std::to_string(i);
                std::string param7 = "CL"+std::to_string(i);

                // Find necessary tag and put variable in array
                RF_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("RF").child(param1.c_str()).text().get()));
                SW_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("SW").child(param2.c_str()).text().get()));
                ADC_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("ADC").child(param3.c_str()).text().get()));
                GX_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("GX").child(param4.c_str()).text().get()));
                GY_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("GY").child(param5.c_str()).text().get()));
                GZ_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("GZ").child(param6.c_str()).text().get()));
                CL_array.push_back(utilityFunctions::to_uint32_t(doc.child("root").child("CL").child(param7.c_str()).text().get()));

                // Condition for correctness
                if(!utilityFunctions::CheckNode(param1, doc.child("root").child("RF").child(param1.c_str())) ||
                                      !utilityFunctions::CheckNode(param2, doc.child("root").child("SW").child(param2.c_str())) ||
                                      !utilityFunctions::CheckNode(param3, doc.child("root").child("ADC").child(param3.c_str())) ||
                                      !utilityFunctions::CheckNode(param4, doc.child("root").child("GX").child(param4.c_str())) ||
                                      !utilityFunctions::CheckNode(param5, doc.child("root").child("GY").child(param5.c_str())) ||
                                      !utilityFunctions::CheckNode(param6, doc.child("root").child("GZ").child(param6.c_str())) ||
                                      !utilityFunctions::CheckNode(param7, doc.child("root").child("CL").child(param7.c_str()))){
                   throw std::invalid_argument("Incorrect XML structure\n");
                }
            }
        }
        catch(const std::invalid_argument& e){
            newlogger << LogPref::Flag(ERROR) << e.what() << '\n';
            return 5;
        }
        newlogger << LogPref::Flag(OK) << "Internal structure of XML is OK\n";

        // Pushing arrays into return vector
        vec.push_back(RF_array);
        vec.push_back(SW_array);
        vec.push_back(ADC_array);
        vec.push_back(GX_array);
        vec.push_back(GY_array);
        vec.push_back(GZ_array);
        vec.push_back(CL_array);

        //doc.save(std::cout);
        return 0;
}

// csv handler
int evalCsv(int argc, char** argv, std::vector<std::vector<uint32_t>>& vec){
        std::string filename = argv[1];
        std::ifstream work_file(filename);
        std::string line;
        char delimiter = ',';
        uint32_t csv_row_counter = 0;
        uint32_t ticks{0};

        std::vector<uint32_t> RF_list;
        std::vector<uint32_t> TR_SW_list;
        std::vector<uint32_t> ADC_list;
        std::vector<uint32_t> GRAD_X_list;
        std::vector<uint32_t> GRAD_Y_list;
        std::vector<uint32_t> GRAD_Z_list;
        std::vector<uint32_t> CYCLES_NUM_list;

        while (getline(work_file, line))
        {
            std::stringstream stream(line);
            std::string RF, TR_SW, ADC, GRAD_X, GRAD_Y, GRAD_Z, CYCLES_NUM, del_;

            std::getline(stream, RF, delimiter);
            std::getline(stream, TR_SW, delimiter);
            std::getline(stream, ADC, delimiter);
            std::getline(stream, GRAD_X, delimiter);
            std::getline(stream, GRAD_Y, delimiter);
            std::getline(stream, GRAD_Z, delimiter);
            std::getline(stream, CYCLES_NUM, delimiter);

            RF_list.push_back(stoi(RF));
            TR_SW_list.push_back(stoi(TR_SW));
            ADC_list.push_back(stoi(ADC));
            GRAD_X_list.push_back(stoi(GRAD_X));
            GRAD_Y_list.push_back(stoi(GRAD_Y));
            GRAD_Z_list.push_back(stoi(GRAD_Z));
            CYCLES_NUM_list.push_back(stoi(CYCLES_NUM));

            std::cout << "==================" << '\n';
            std::cout << "RF: " << RF << '\n';
            std::cout << "TR_SW: " << TR_SW << '\n';
            std::cout << "ADC: " << ADC << '\n';
            std::cout << "GRAD_X: " << GRAD_X << '\n';
            std::cout << "GRAD_Y: " << GRAD_Y << '\n';
            std::cout << "GRAD_Z: " << GRAD_Z << '\n';
            std::cout << "CYCLES_NUM: " << CYCLES_NUM<< '\n';

            csv_row_counter += 1;
            ticks += stoi(CYCLES_NUM);

        }
        work_file.close();
//    auto iter_rf = RF_list.begin();
//    auto iter_tr_sw = TR_SW_list.begin();
//    auto iter_adc = ADC_list.begin();
//    auto iter_grad = GRAD_list.begin();
//    auto iter_cycles = CYCLES_NUM_list.begin();

//    for (int i=0; i<csv_row_counter;i++){
//        cout << *iter_rf;
//        iter_rf++;
//    }
    //end of reading pulse sequence from csv file
        vec.push_back(RF_list);
        vec.push_back(TR_SW_list);
        vec.push_back(ADC_list);
        vec.push_back(GRAD_X_list);
        vec.push_back(GRAD_Y_list);
        vec.push_back(GRAD_Z_list);
        vec.push_back(CYCLES_NUM_list);

    return 0;
}

int main(int argc, char** argv)
{
    // Default console enabled
    newlogger.enableConsoleOutput(true);

    // Flag arguments handler
    if(argc == 1)
    {
        std::cout << "ERROR: no args (use Sync.exe <filename>)" << endl;
        return -1;
    }
    if(argc > 2)
    {
    for(int i = 2; i < argc; i++)
    {
        if(strcmp(argv[i], "--debug")==0)
        {
            en_debug = true;
        }
        else if(strcmp(argv[i], "--disable-console")==0)
        {
            newlogger.enableConsoleOutput(false);
        }
    }
    }
    int status = INFO;

    // Default duepp debug string
    strcpy(dstr,"duepp: Everything is allright\n");

// READING SEQUENCE FILE
    pugi::xml_document doc;
    newlogger << "Start...\n";
    std::string fileName = argv[1];
    newlogger << "File: " << fileName << "\n";
    size_t fileNameSize = fileName.size();

	std::vector<std::vector<uint32_t>> returnmentValues;

	try {
        if(fileName.substr(fileNameSize - 3, 3) == "xml") evalXml(argc, argv, doc, returnmentValues);
        else if(fileName.substr(fileNameSize - 3, 3) == "csv") evalCsv(argc, argv, returnmentValues);
        else {
            throw std::invalid_argument("Incorrect file name or size\n");
        }
	}
	catch(const std::invalid_argument& e){
		newlogger << LogPref::Flag(ERROR) << e.what() << '\n';
		return 5;
	}

	newlogger << LogPref::Flag(DONE) << "File loaded successfully\n";

	uint32_t ParamCount =  utilityFunctions::to_uint32_t(doc.child("root").child("ParamCount").text().get());

    /*const string filepath{""};
    const string filename{"sync_v2.xml"};

    newlogger << filepath+filename << endl;

    pugi::xml_document doc;

    pugi::xml_parse_result result = doc.load_file((filepath+filename).c_str());

    newlogger << boolalpha << showpoint; //boolalpha and noboolalpha

    newlogger << "Load result: " << result.description() << std::endl;

//    CheckNode("noparam", doc.child("config").child("nochild") );
//    CheckNode("param1", doc.child("config").child("param1") );
//    CheckNode("param2", doc.child("config").child("param2") );
//    CheckNode("param3", doc.child("config").child("param3") );
//    CheckNode("param4", doc.child("config").child("param4") );
//    CheckNode("param5", doc.child("config").child("param5") );
//    CheckNode("param6", doc.child("config").child("param6") );
//
//    CheckNode("config2-param1", doc.child("config").child("config2").child("param1") );

    newlogger<<"ParamCount = ";

    uint32_t ParamCount =  to_uint32_t(doc.child("root").child("ParamCount").text().get());
    newlogger<<ParamCount<<std::endl;
    int *RF_array {new int [ParamCount]{}};
    int *SW_array {new int [ParamCount]{}};
    int *ADC_array {new int [ParamCount]{}};
    //int *GRU_array {new int [ParamCount]{}};
    int *CL_array {new int [ParamCount]{}};

    for (int i = 1; i <= ParamCount; i++){

        string param1 {"RF"+to_string(i)};
        string param2 {"SW"+to_string(i)};
        string param3 {"ADC"+to_string(i)};
        string param4 {"GRU"+to_string(i)};
        string param5 {"CL"+to_string(i)};
//        newlogger << param1;

        RF_array[i] = to_uint32_t(doc.child("root").child("RF").child(param1.c_str()).text().get());
        SW_array[i] = to_uint32_t(doc.child("root").child("SW").child(param2.c_str()).text().get());
        ADC_array[i] = to_uint32_t(doc.child("root").child("ADC").child(param3.c_str()).text().get());
        //GRU_array[i] = to_uint32_t(doc.child("root").child("GRU").child(param4.c_str()).text().get());
        CL_array[i] = to_uint32_t(doc.child("root").child("CL").child(param5.c_str()).text().get());

        CheckNode(param1, doc.child("root").child("RF").child(param1.c_str()));
        CheckNode(param2, doc.child("root").child("SW").child(param2.c_str()));
        CheckNode(param3, doc.child("root").child("ADC").child(param3.c_str()));
        //CheckNode(param4, doc.child("root").child("GRU").child(param4.c_str()));
        CheckNode(param5, doc.child("root").child("CL").child(param5.c_str()));
    }//for
    doc.save(out);*/

// Default output file
const string OutFilePath {"D:\Synchronisation\dueppwinserial"};
const string OutFileName {"0000"};
const string OutFileNameCommand {"_command"};
const string OutFileNameData {"_data"};
string OutFile;

    newlogger << "Creating synchronization program..." << endl;

    outputs1 = 0x02000000;
    ticks1 = 0x000000FF;

    outputs2 =  0x00000000;
    ticks2 =  0x000000FF;

    //reps=3;

    reps = 100;

    //test with rf
    uint32_t time_high =1000; //time in us
    uint32_t time_low =1000; //time in us
    uint32_t time_delay =50; //time in us
    uint32_t time_low_corrected =1000; //time in us

    uint32_t out_high =0x00000002; //pin outputs
    uint32_t out_low = 0x00000010; //pin outputs
    uint32_t ticks_high = time_high*1000/20; //number of 20 ns ticks when pins are high
    uint32_t ticks_low_corrected = time_low_corrected*1000/20; //number of 20 ns ticks when pins are high

    //uint8_t ticks_high = 0x2FAF080;
    uint32_t ticks_low = time_low*1000/20; //number of 20 ns ticks when pins are low
    uint32_t ticks_delay = time_delay*1000/20; //number of 20 ns ticks when pins are high
    uint32_t ticks_low_mod = ticks_low - 2*ticks_delay;

    due_prog_t program1;

    due_init_program(&program1, 0);


/*
//example
    due_add_event(&program1, outputs1, ticks1);
    due_add_event(&program1, outputs2, ticks2);
    //due_wait_for_trigger(&program1, outputs2, ticks2);

    for (int ii=0; ii<reps; ii++){
        due_add_event(&program1, outputs1, ticks1);

        due_add_event(&program1, outputs2, ticks2);
    }


*/
/*
    due_add_event(&program1, outputs2, 20);
    due_wait_for_trigger(&program1, out_high, ticks_high);


    for (int ii=0; ii<reps; ii++){
        due_add_event(&program1, out_low, ticks_low);
        due_add_event(&program1, out_high, ticks_high);
    }*/

/*
    due_start_loop(&program1, out_high, ticks_high, reps);//порядок параметров поменять
    due_end_loop(&program1, out_low, ticks_low);*/

    // Get outputs configuration for TTL signal
    uint32_t XML_outputs{0};

    for(int i=0; i<ParamCount;i++){
        XML_outputs = GetOutputs(returnmentValues[0][i],returnmentValues[1][i],returnmentValues[2][i],0);

        // Redirecting stdout from due_pp_lib to string
        freopen("temp", "w", stdout);
        setbuf(stdout, dstr);
            status = due_add_event(&program1, XML_outputs,returnmentValues[6][i]);
        freopen("CON", "w", stdout);
        if(en_debug)
            newlogger << LogPref::Flag(DEBUG) << "Analise Node " << i << "/" << ParamCount << "..." << std::endl;
            newlogger << LogPref::Flag(status+DEBUG) << dstr;
        strcpy(dstr,"duepp: Everything is allright\n");
    }

    newlogger << LogPref::Flag(DONE) << "Events added" << endl;

    freopen("temp", "w", stdout);
    setbuf(stdout, dstr);
        status = due_exit_program(&program1);
    freopen("CON", "w", stdout);
    if(en_debug)
        newlogger << LogPref::Flag(status+DEBUG) << dstr;
    strcpy(dstr,"duepp: Everything is allright\n");

    freopen("temp", "w", stdout);
    setbuf(stdout, dstr);
        status = due_finalize_program(&program1);
    freopen("CON", "w", stdout);
    if(en_debug)
        newlogger << LogPref::Flag(status+DEBUG) << dstr;
    strcpy(dstr,"duepp: Everything is allright\n");

    freopen("temp", "w", stdout);
    setbuf(stdout, dstr);
        status = due_dump_program(&program1);
    freopen("CON", "w", stdout);
    dstr[strlen(dstr)-1] = '\0'; // There is strange nosense 4, so I cut it
    dstr[strlen(dstr)-1] = '\0';
    if(en_debug)
        newlogger << LogPref::Flag(status+DEBUG) << "duepp: Get program dump:\n" << dstr << endl;
    strcpy(dstr,"duepp: Everything is allright\n");

    // Getting hex program
    stringstream ss;
    for (int32_t ii=0;  ii<program1.dpos+0; ii++)
    {
        //newlogger << "" << std::hex << std::showbase << std::uppercase << program1.data[ii]<< endl;
        ss << "" << std::hex << std::setfill('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< endl;
    }  //for

    if(en_debug)
        newlogger << "Program in HEX:\n" << ss.str();

    /*-----------------------
    newlogger << "" << endl;
    newlogger << "trying due_download_prog_save_to_file() " << endl;

   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC);  //! creates readonly file - wrong permissions
   //int fd = open("d:\\work\\codeblocks projects\\dueppwin00\\0000", O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   OutFile = OutFilePath + OutFileName;

   int fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

   newlogger << std::dec << "file fd = " << fd << endl;

    //due_download_prog(fd, &program1);
    due_download_prog_save_to_file(fd, &program1);

    close(fd);

   //-----------------------*/

   // Saving synchroprogram to output file
    newlogger << "Trying due_download_prog_save_to_file_command() " << endl;

    OutFile = std::string("");

    newlogger << std::dec << "file fd = " << 1 << endl;
    int fd = open(OutFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);  //ok

    //due_download_prog(fd, &program1);

    freopen("temp", "w", stdout);
    setbuf(stdout, dstr);
        status = due_download_prog_save_to_file_command(fd, &program1);
    freopen("CON", "w", stdout);
    dstr[strlen(dstr)-1] = '\0'; // There is strange nosense 4, so I cut it
    dstr[strlen(dstr)-1] = '\0';
    newlogger << LogPref::Flag(status+DEBUG) << dstr << endl;
    strcpy(dstr,"duepp: Everything is allright\n");

    close(fd);


    //Opening serial port
    newlogger << "Creating Serial object" << endl;

     // Serial object
    serialib serial;

    newlogger << "Connecting to Serial object" << endl;

    // Connection to serial port
    //char errorOpening = serial.openDevice(SERIAL_PORT, 38400);
    char errorOpening = serial.openDevice(SERIAL_PORT.c_str(), 38400);


    newlogger << "Checking connection" << endl;


    // If connection fails, return the error code otherwise, display a success message
    if (errorOpening!=1)
    {
        newlogger << LogPref::Flag(ERROR) << "No connection to " << SERIAL_PORT << std::endl;

        return errorOpening;
    }
    else  newlogger << LogPref::Flag(DONE) << "Successful connection to " << SERIAL_PORT.c_str();

    serial.flushReceiver();

    serial.writeChar('S');
    //  this_thread::sleep_for(100ms);
    this_thread::sleep_for(100ms);

    ii = serial.available();
    newlogger << "S serial.available = " << ii<< endl;

    serial.readString(buffer, '\n', 254, 1000);
    newlogger << "S serial.readString = " << buffer << endl;


    serial.writeChar('Q');
//  this_thread::sleep_for(100ms);

   ii = serial.available();
   newlogger << "Q serial.available = " << ii<< endl;

    //serial.readChar(&ch, 1000);
    //printf ("ch = %c\n", ch);
    //newlogger << " serial.readChar = " <<  ch << endl;

    serial.readString(buffer, '\n', 254, 1000);
    newlogger << "Q serial.readString = " << buffer << endl;

//// send data length, low byte, high byte
//  cbyte[0] = 'D';
//  cbyte[1] = program->dpos & 0xff;
//  cbyte[2] = (program->dpos>>8)&0xff;
//  write(fd,cbyte,3);

//serial.writeChar('D');
//serial.writeChar(char(program1.dpos & 0xff));
//serial.writeChar(char((program1.dpos>>8)&0xff));
//
//b[0] = uint8_t('D');
//newlogger<<"byte0 = ";
//newlogger<<char(b[0])<<"  ";
//newlogger << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[0]) << endl;
//
//b[1] = uint8_t((program1.dpos>>0)&0xff);
////    newlogger << "" << setbase(16) << std::setfill ('0') << std::setw(8)<< std::uppercase << program1.data[ii]<< endl;
//newlogger<<"byte1 = ";
////newlogger<<b[0]<<"  ";
//newlogger << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[1]) << endl;
////serial.writeChar((b));
////serial.writeBytes(b, 1);
//
//b[2] = uint8_t((program1.dpos>>8)&0xff);
//newlogger<<"byte2 = " ;
////newlogger<<b[0]<<"  ";
//newlogger << setbase(16) <<  std::setfill ('0') << std::setw(2)<< std::uppercase << int16_t(b[2]) << endl;
////serial.writeChar(char(b[0]));
////serial.writeBytes(b, 1);
//
//serial.writeBytes(b, 3);
int due_dl{0};


freopen("temp", "w", stdout);
setbuf(stdout, dstr);
    due_dl = due_upload_trajectory(serial, &program1);
freopen("CON", "w", stdout);
if(en_debug)
    newlogger << LogPref::Flag(due_dl+DEBUG) << dstr;
strcpy(dstr,"duepp: Everything is allright\n");
//newlogger << std::dec << "" << endl;
//
//   ii = serial.available();
//    newlogger << "D serial.available = " << ii<< endl;
//
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "D serial.readString = " << buffer << endl;
//
//    serial.writeBytes(program1.data, program1.dpos*4);
////    this_thread::sleep_for(100ms);
//
//    ii = serial.available();
//    newlogger << "data serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "data serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   newlogger << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "S serial.readString = " << buffer << endl;
//
////   serial.writeChar('E');
//////   serial.writeChar('e');
//////  this_thread::sleep_for(100ms);
////   ii = serial.available();
////   newlogger << "e serial.available = " << ii<< endl;
////    serial.readString(buffer, '\n', 254, 1000);
////    newlogger << "e serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   newlogger << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "S serial.readString = " << buffer << endl;
//
//serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   newlogger << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "S serial.readString = " << buffer << endl;
//
// serial.writeChar('S');
////  this_thread::sleep_for(100ms);
//   ii = serial.available();
//   newlogger << "S serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "S serial.readString = " << buffer << endl;
//
//    newlogger << "" << endl;
//    newlogger << "" << endl;
//    this_thread::sleep_for(1000ms);
//
//
//
//       serial.writeChar('E');
//////   serial.writeChar('e');
//
//   ii = serial.available();
//   newlogger << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "e serial.readString = " << buffer << endl;
//
//
//        this_thread::sleep_for(1000ms);
//
//
//
//       serial.writeChar('E');
//////   serial.writeChar('e');
//
//   ii = serial.available();
//   newlogger << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "e serial.readString = " << buffer << endl;
//
//       ii = serial.available();
//   newlogger << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "e serial.readString = " << buffer << endl;
//
//
//       ii = serial.available();
//   newlogger << "e serial.available = " << ii<< endl;
//    serial.readString(buffer, '\n', 254, 1000);
//    newlogger << "e serial.readString = " << buffer << endl;


        //this_thread::sleep_for(30000ms);

    newlogger << LogPref::Flag(OK) << "DONE!" << endl;
    return 0;
}
