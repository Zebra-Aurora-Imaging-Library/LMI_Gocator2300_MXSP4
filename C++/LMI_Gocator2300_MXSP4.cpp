//***************************************************************************************/
// 
// File name: LMI_Gocator2300_MXSP4.cpp  
//
// Synopsis:  This program contains an example of 3d reconstruction by interfacing with
//            a LMI Gocator 2330 scanner.
//            See the PrintHeader() function below for detailed description.
//
// Copyright © 1992-2024 Zebra Technologies Corp. and/or its affiliates
// All Rights Reserved
//***************************************************************************************/

#include <mil.h>
#include <vector>

#include "MainTemplate.h"
// Once the Gocator SDK is installed and the project is configured correctly (see
// PrintHeader()), set GOSDK_INSTALLED to 1 to enable the Gocator-specific code.
// You need an actual Gocator connected to your computer.
#define GOSDK_INSTALLED  0

//****************************************************************************
// Example description.
//***************************************************************************
void PrintHeader()   
   {
   MosPrintf(MIL_TEXT("[EXAMPLE NAME]\n")
             MIL_TEXT("LMI_Gocator2300_MXSP4\n\n")

             MIL_TEXT("[SYNOPSIS]\n")
             MIL_TEXT("This program acquires a 3d point cloud using an LMI Gocator 2330 sensor\n")
             MIL_TEXT("with the Gocator API. It then converts the point cloud to the MIL\n")
             MIL_TEXT("format and displays the result.\n\n")

             MIL_TEXT("[MODULES USED]\n")
             MIL_TEXT("Modules used: application, system, buffer, 3D Display,\n")
             MIL_TEXT("              3D Graphics, 3D Processing.\n\n"));
   }

//*****************************************************************************
// Gocator-specific header, constants and helper functions
//*****************************************************************************

#if GOSDK_INSTALLED
   // The project options must be modified so that this header is reachable.
   #include <GoSdk/GoSdk.h>

   // World units converters.
   #define NM_TO_MM(VALUE)    (((k64f)(VALUE))/1000000.0)
   #define UM_TO_MM(VALUE)    (((k64f)(VALUE))/1000.0)

   // Change this to your Gocator's address, if different.
   static const kChar* CAM_IP_ADDRESS = reinterpret_cast<const kChar*>("192.168.1.110");

   // Timeout related to acquisition time.
   static const k64u RECEIVE_TIMEOUT = 20000000;  // in us

   // Helper function for SAFE_GOSDK_CALL.
   bool GoSdkCallSucceeded(kStatus CamStatus, const MIL_TEXT_CHAR* FunctionName)
      {
      if (!kSuccess(CamStatus))
         {
         MosPrintf(MIL_TEXT("Error in '%s' (status: %d)\n"), FunctionName, static_cast<int>(CamStatus));
         MosPrintf(MIL_TEXT("\nress <Enter> to end.\n"));
         MosGetch();
         return false;
         }
      return true;
      }
   
   //*****************************************************************************
   // Pass the status of each GoSdk call to this macro. It will print an error
   // with the given function name and exit the current function if the GoSdk
   // function failed.
   //
   //   CamStatus       (in)  Return status of the GoSdk call.
   //   FunctionName    (in)  Name of the function to print in the error message.
   //*****************************************************************************
   #define SAFE_GOSDK_CALL(CamStatus, FunctionName)  \
      if (!GoSdkCallSucceeded(CamStatus, FunctionName)) return
#endif

// Helper function for EXAMPLE_ASSERT.
bool AssertSucceeded(bool Condition, const MIL_TEXT_CHAR* Message)
   {
   if (!Condition)
      {
      MosPrintf(Message);
      MosPrintf(MIL_TEXT("\nPress <Enter> to end.\n"));
      MosGetch();
      return false;
      }
   return true;
   }
   
//*****************************************************************************
// Tests necessary conditions using this macro. It will print an error
// with the given string and exit the current function if the condition
// is false.
//
//   Condition       (in)  The condition assumed to be true.
//   Message         (in)  Error message.
//*****************************************************************************
#define EXAMPLE_ASSERT(Condition, Message)   if (!AssertSucceeded(Condition, Message)) return

//*****************************************************************************
// Structure containing objects that need deallocation (MIL and GoSdk, if enabled).
//*****************************************************************************
struct SObjects
   {
   MIL_ID MilApplication;  // Application identifier
   MIL_ID MilSystem;       // System identifier
   MIL_ID MilPtCldCtn;     // Point cloud container

#if GOSDK_INSTALLED
   kAssembly   CamApi;     // Gocator API library
   GoSystem    CamSystem;  // System of Gocator devices
   GoDataSet   CamDataSet; // Collection of Gocator data channels.
#endif
   };

//*****************************************************************************
// Structure representing a 3d point cloud, with position and intensity data.
//*****************************************************************************
struct SPointCloud
   {
   std::vector<MIL_INT16>  z; // Point cloud position data
   std::vector<MIL_UINT8 > i;       // Point cloud intensity data

   void SetIntensitySize(MIL_INT NumElements) { i.resize(NumElements); };
   void SetPositionSize (MIL_INT NumElements)
      {
      z.resize(NumElements);
      }
   };

//*****************************************************************************
// Helper function prototypes
//*****************************************************************************

void AcquirePointCloudData(SObjects* ObjectsPtr);
void DisplayPointCloudData(MIL_ID MilSystem, MIL_ID MilContainer);

//*****************************************************************************
// Main.
//*****************************************************************************
int MosMain()
   {
   SObjects Objects;
   memset(&Objects, 0, sizeof(Objects));

   PrintHeader();

#if !GOSDK_INSTALLED
   MosPrintf(MIL_TEXT("This example is designed to be used with an LMI Gocator 3d sensor and\n"));
   MosPrintf(MIL_TEXT("the Gocator SDK. To run the example:\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Install the Gocator SDK.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Follow the Gocator Quick Start Guide:\n"));
   MosPrintf(MIL_TEXT("  - Connect the Gocator sensor to your computer.\n"));
   MosPrintf(MIL_TEXT("  - Change your network adapter settings.\n"));
   MosPrintf(MIL_TEXT("  - Connect to the Gocator sensor using a web browser.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Use the web browser interface to configure and calibrate the Gocator sensor:\n"));
   MosPrintf(MIL_TEXT("  - In the Scan pane, configure the settings according to your setup.\n"));
   MosPrintf(MIL_TEXT("  - In the Output pane, ensure that the Ethernet protocol is set to Gocator\n"));
   MosPrintf(MIL_TEXT("    and that the Surfaces (and optionally, the Surface Intensities) check boxes are checked.\n"));
   MosPrintf(MIL_TEXT("  - Refer to the Gocator User Guide for more information.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- If necessary, compile the GoSdk and kApi shared libraries for your platform.\n"));
   MosPrintf(MIL_TEXT("  Refer to the Gocator API Manual for more information. The following instructions\n"));
   MosPrintf(MIL_TEXT("  assume a release build.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- An environment variable named GO_SDK should exist, with its value corresponding\n"));
   MosPrintf(MIL_TEXT("  to the path of the GO_SDK folder.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Copy the shared libraries to your system path. Under Windows, copy\n"));
   MosPrintf(MIL_TEXT("    $(GO_SDK)\\bin\\<platform>\\GoSdk.dll\n"));
   MosPrintf(MIL_TEXT("    $(GO_SDK)\\bin\\<platform>\\kApi.dll\n"));
   MosPrintf(MIL_TEXT("  to\n"));
   MosPrintf(MIL_TEXT("    %%SYSTEMROOT%%\\System32\\   (usually C:\\Windows\\System32\\)\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Add the paths to the header files and library files of the Gocator SDK to\n"));
   MosPrintf(MIL_TEXT("  the example project files. If you are using Visual Studio, open the Property\n"));
   MosPrintf(MIL_TEXT("  Pages of the LMI_Gocator2300_MXSP4 project. Then, under Configuration Properties,\n"));
   MosPrintf(MIL_TEXT("  you must:\n"));
   MosPrintf(MIL_TEXT("  - Add\n"));
   MosPrintf(MIL_TEXT("      $(GO_SDK)\\Gocator\\GoSdk \n"));
   MosPrintf(MIL_TEXT("      $(GO_SDK)\\Platform\\kApi \n"));
   MosPrintf(MIL_TEXT("    to\n"));
   MosPrintf(MIL_TEXT("      C/C++->General->Additional Include Directories\n"));
   MosPrintf(MIL_TEXT("  - Add \n"));
   MosPrintf(MIL_TEXT("      $(GO_SDK)\\lib\\<platform> \n"));
   MosPrintf(MIL_TEXT("    to \n"));
   MosPrintf(MIL_TEXT("      Linker->General->Additional Library Directories\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Link with the Gocator libraries. If you are using Visual Studio, open the\n"));
   MosPrintf(MIL_TEXT("  Property Pages of the LMI_Gocator2300_MXSP4 project. Then, under Configuration\n"));
   MosPrintf(MIL_TEXT("  Properties, you must:\n"));
   MosPrintf(MIL_TEXT("  - Add both GoSdk.lib and kApi.lib to Linker->Input->Additional Dependencies\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Update the example code:\n"));
   MosPrintf(MIL_TEXT("  - Set the GOSDK_INSTALLED define to 1.\n"));
   MosPrintf(MIL_TEXT("  - Change the CAM_IP_ADDRESS variable, if necessary.\n"));
   MosPrintf(MIL_TEXT("  - Recompile the example.\n"));
   MosPrintf(MIL_TEXT("\n\n"));
   MosPrintf(MIL_TEXT("The example has been tested with the following setups:\n"));
   MosPrintf(MIL_TEXT("- Windows 10 64-bit and VS2017.\n"));
   MosPrintf(MIL_TEXT("- Gocator 2330.\n"));
   MosPrintf(MIL_TEXT("- Gocator API (version 5.2.19.72).\n"));
   MosPrintf(MIL_TEXT("- MIL 10 Sp4 Build 602 (EA).\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
   MosGetch();
#else

   // Acquire and display Gocator's 3d point cloud data.
   AcquirePointCloudData(&Objects);

   // Free Gocator data set.
   if (Objects.CamDataSet != kNULL)
      GoDestroy(Objects.CamDataSet);        

   // Free Gocator system.
   if (Objects.CamSystem != kNULL)
      {
      GoSystem_Stop(Objects.CamSystem); 
      GoDestroy(Objects.CamSystem);     
      }

   // Free Gocator API Library.
   if (Objects.CamApi != kNULL)
      GoDestroy(Objects.CamApi);     

#endif

   // Cleanup MIL objects.
   if (Objects.MilPtCldCtn     != M_NULL) MbufFree  (Objects.MilPtCldCtn    );
   if (Objects.MilSystem       != M_NULL) MsysFree  (Objects.MilSystem      );
   if (Objects.MilApplication  != M_NULL) MappFree  (Objects.MilApplication );
   return 0;
   }


//*****************************************************************************
// Initializes the Gocator, acquires a 3d point cloud, converts it to the MIL 
// format and shows the result.
//
//   ObjectsPtr (in-out)  Structure containing all objects needing deallocation
//*****************************************************************************
void AcquirePointCloudData(SObjects* ObjectsPtr)
   {
   // Information about the 3d point cloud data.
   SPointCloud PointCloud;
   bool        GotPositionData  = false;
   bool        GotIntensityData = false;
   MIL_INT     NumPositionRows  = 0;
   MIL_INT     NumPositionCols  = 0;
   MIL_INT     NumIntensityRows = 0;
   MIL_INT     NumIntensityCols = 0;
   MIL_INT     NumPoints        = 0;

   MIL_DOUBLE  XResolution      = 0;
   MIL_DOUBLE  YResolution      = 0;
   MIL_DOUBLE  ZResolution      = 0;
   MIL_DOUBLE  XOffset          = 0;
   MIL_DOUBLE  YOffset          = 0;
   MIL_DOUBLE  ZOffset          = 0;

   // The Gocator transmits range data as 16-bit signed integers. Invalid
   // data corresponds to the minimum 16-bit signed integer value.
   static const MIL_INT INVALID_RANGE_16BIT = -32768L;

   // Allocate and initialize MIL application objects.
   MappAlloc(M_NULL, M_DEFAULT, &ObjectsPtr->MilApplication);
   MsysAlloc(ObjectsPtr->MilApplication, M_SYSTEM_HOST, M_DEFAULT, M_DEFAULT, &ObjectsPtr->MilSystem);

   MosPrintf(MIL_TEXT("The Gocator 3d sensor will be initialized to acquire a 3d point cloud.\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to continue.\n\n"));
   MosGetch();

#if GOSDK_INSTALLED

   // ------------------------------------------------------------------------
   // Gocator initialization.

   // Allocate and initialize GoSdk objects.
   kIpAddress CamIpAddress;
   GoSensor   CamSensor;
   SAFE_GOSDK_CALL( GoSdk_Construct(&ObjectsPtr->CamApi)                          , MIL_TEXT("GoSdk_Construct")                 ); // Construct Gocator API Library
   SAFE_GOSDK_CALL( GoSystem_Construct(&ObjectsPtr->CamSystem, kNULL)             , MIL_TEXT("GoSystem_Construct")              ); // Construct Gocator system object
   SAFE_GOSDK_CALL( kIpAddress_Parse(&CamIpAddress, CAM_IP_ADDRESS)               , MIL_TEXT("kIpAddress_Parse")                ); // Parse IP address into address data structure
   SAFE_GOSDK_CALL( GoSystem_FindSensorByIpAddress(ObjectsPtr->CamSystem, 
                                                   &CamIpAddress, &CamSensor)     , MIL_TEXT("GoSystem_FindSensorByIpAddress")  ); // Obtain Gocator sensor object by IP address
   SAFE_GOSDK_CALL( GoSensor_Connect(CamSensor)                                   , MIL_TEXT("GoSensor_Connect")                ); // Create connection to Gocator sensor object
   SAFE_GOSDK_CALL( GoSystem_EnableData(ObjectsPtr->CamSystem, kTRUE)             , MIL_TEXT("GoSystem_EnableData")             ); // Enable data channel
   
   // Setup the 3d sensor.
   GoSetup CamSetup = GoSensor_Setup(CamSensor);                                                                                             
   
   // Start the 3d sensor.
   SAFE_GOSDK_CALL( GoSystem_Start(ObjectsPtr->CamSystem)                         , MIL_TEXT("GoSystem_Start")                  ); // Start Gocator system

   // ------------------------------------------------------------------------
   // 3d point cloud data acquisition.
   MosPrintf(MIL_TEXT("Acquiring 3d point cloud data... "));

   // Receive the 3d point cloud data. If no data is received after a timeout,
   // the function will return.
   SAFE_GOSDK_CALL( GoSystem_ReceiveData(ObjectsPtr->CamSystem, &ObjectsPtr->CamDataSet, RECEIVE_TIMEOUT), MIL_TEXT("GoSystem_ReceiveData") );

   // Each result can have multiple data items. Loop through all items
   // in result message.
   for (MIL_UINT iSet = 0; iSet < GoDataSet_Count(ObjectsPtr->CamDataSet); ++iSet)
      {         
      GoDataMsg DataMsg = GoDataSet_At(ObjectsPtr->CamDataSet, iSet);

      switch (GoDataMsg_Type(DataMsg))
         {
           case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:// 3d position data
            {   
            GotPositionData = true;

            GoSurfaceMsg SurfaceMsg = DataMsg;

            // Get the point cloud data size.
            NumPositionRows = GoSurfaceMsg_Length(SurfaceMsg);
            NumPositionCols = GoSurfaceMsg_Width(SurfaceMsg);
            NumPoints = NumPositionRows * NumPositionCols;

            // Get calibration information for converting range data to 3d point cloud data.
            XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(SurfaceMsg));
            YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(SurfaceMsg));
            ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(SurfaceMsg));
            XOffset     = UM_TO_MM(GoSurfaceMsg_XOffset    (SurfaceMsg));
            YOffset     = UM_TO_MM(GoSurfaceMsg_YOffset    (SurfaceMsg));
            ZOffset     = UM_TO_MM(GoSurfaceMsg_ZOffset    (SurfaceMsg));
            
            // Allocate point cloud position data memory.
            PointCloud.SetPositionSize(NumPoints);

            // Convert range map to 3d point cloud data.
            MIL_INT iPoint = 0;
            for(MIL_INT iRow = 0; iRow < NumPositionRows; iRow++)
               {
               k16s* DataRowPtr = GoSurfaceMsg_RowAt(SurfaceMsg, iRow);
                  
               for(MIL_INT iCol = 0; iCol < NumPositionCols; iCol++)
                  {
                     PointCloud.z[iPoint] = DataRowPtr[iCol];
                     // Increment point index.
                     iPoint++;
                  }
               }               
            }       
            break;

         case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY: // Intensity data
            {   
            GotIntensityData = true;

            GoSurfaceIntensityMsg IntensityMsg = DataMsg;

            // Get the intensity data size.
            NumIntensityRows = GoSurfaceIntensityMsg_Length(IntensityMsg);
            NumIntensityCols = GoSurfaceIntensityMsg_Width(IntensityMsg);
            NumPoints = NumIntensityRows * NumIntensityCols;

            // Allocate point cloud intensity data memory.
            PointCloud.SetIntensitySize(NumPoints);

            // Get 3d point cloud intensity data.
            MIL_INT iPoint = 0;
            for(MIL_INT iRow = 0; iRow < NumIntensityRows; iRow++)
               {
               k8u* DataRowPtr = GoSurfaceIntensityMsg_RowAt(IntensityMsg, iRow);
                  
               for(MIL_INT iCol = 0; iCol < NumIntensityCols; iCol++)
                  // Copy intensity value and increment point index.
                  PointCloud.i[iPoint++] = DataRowPtr[iCol];
               }               
            }       
            break;

         default:
            // Ignore the other results.
            break;
         }
      }
#endif

   // Check that 3d point cloud data was actually received.
   EXAMPLE_ASSERT(GotPositionData, MIL_TEXT("Data received, but without any part data.\n")
                                   MIL_TEXT("No Ethernet output enabled?\n"));

   // Check that positional and intensity data (if available) have equal sizes.
   if (GotIntensityData)
      EXAMPLE_ASSERT((NumPositionRows == NumIntensityRows) && (NumPositionCols == NumIntensityCols), 
                     MIL_TEXT("Positional data size differs from intensity data size.\n"));

   MosPrintf(MIL_TEXT("OK.\n\n"));

   //---------------------------------------------------------------------------
   MbufAllocContainer(ObjectsPtr->MilSystem, M_PROC + M_DISP, M_DEFAULT, &ObjectsPtr->MilPtCldCtn);
   
   MIL_ID MilRangeComponent = MbufInquireContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_COMPONENT_ID, M_NULL);
   MbufAllocComponent(ObjectsPtr->MilPtCldCtn, 1, NumPositionCols, NumPositionRows, 16 + M_SIGNED, M_IMAGE + M_PROC, M_COMPONENT_RANGE, &MilRangeComponent);
   MbufPut(MilRangeComponent, &PointCloud.z[0]);

   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_INVALID_DATA_FLAG, M_TRUE);
   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_INVALID_DATA_VALUE, INVALID_RANGE_16BIT);

   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_SCALE_X, XResolution);
   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_SCALE_Y, YResolution);
   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_SCALE_Z, ZResolution);

   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_OFFSET_X, XOffset);
   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_OFFSET_Y, YOffset);
   MbufControlContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_RANGE, M_3D_OFFSET_Z, ZOffset);

   MIL_ID MilIntensityComponent = MbufInquireContainer(ObjectsPtr->MilPtCldCtn, M_COMPONENT_INTENSITY, M_COMPONENT_ID, M_NULL);
   MbufAllocComponent(ObjectsPtr->MilPtCldCtn, 1, NumPositionCols, NumPositionRows, 8 + M_UNSIGNED, M_IMAGE + M_PROC, M_COMPONENT_INTENSITY, &MilIntensityComponent);
   MbufPut2d(MilIntensityComponent, 0, 0, NumIntensityCols, NumIntensityRows,&PointCloud.i[0]);
 
   DisplayPointCloudData(ObjectsPtr->MilSystem, ObjectsPtr->MilPtCldCtn);
   }
