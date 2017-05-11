Open Source Overview
============================
CACC for Aimsun is designed to simulate close-spaced CACC platoons in Aimsun via a custom behavioral model developed with microSDK.
This application can be used to evaluate the operational benefits of Cooperative Adaptive Cruise Control on freeways.

License information
-------------------
Licensed under the Aimsun End-User License Agreement. You may not use this code except in compliance with the License. You can request a copy of the License by writing to info@aimsun.com
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

System Requirements
-------------------------
CACC for Aimsun requires an Aimsun microSDK license to be compiled, and an Aimsun license to be executed. The requirements for the use of this application are the same as those for Aimsun alone; see https://www.aimsun.com/aimsun/tech-specs/ for details.

Aimsun runs on Windows, Linux, or Mac, and this application can be compiled for any of those operating systems.

This custom behavioral model has been developed for Aimsun 8.0. Visual Studio 2005 is required to compile it under Windows.

Documentation
-------------
Compile the provided code to produce a .dll or .dylib. Copy the library and the provided .xml in the folder where custom behavioral models are stored (for example, in Windows: Aimsun 8.0\plugins\aimsun\model).
	-	CACC::Model (integer)
	Model attributes define which car following model will be applied, the possible values are:
	-	CACC::TargetHeadway (double)
The car following model uses the Target Headway attribute as the desired distance.
	-	CACC::SpeedFactor (double)
	Acceleration, distance and speed factors are values from 0.0 to 1.0 and they are used to calibrate the model. They set the weights at each component (acceleration, speed and distance). These weights are useful to correct each component, i.e. increasing the speed weight will make the follower to reduce the speed difference respect their leader. Acceleration factor has only effect on the first CACC model.
	-	CACC::MaxHeadway (double)
The maximum headway determines how far a vehicle will be considered as a possible leader when trying to form a platoon. This only affects in Lane Changing model.
	-	CACC::Veh2 (integer)
CACCVeh2 and CACCVeh5 are the Aimsun CACC vehicle identifiers used in the demand.
LaneTypeId sets the CACC reserved lane type identifier. CACC model will only be applied on vehicles in this lane type

	Road Type:

Web sites
---------
The CACC for Aimsun software is distributed through the USDOT's JPO Open Source Application Development Portal (OSADP)
http://itsforge.net/ 