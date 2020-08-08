using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Profiling;

// TODO: floats vs double vs int, Vector3 vs Vector3d, Mathf vs Math
// TODO: check memory fragmentation
// TODO: compare profiler results

namespace MuMech
{
	public class MechJebModuleRoverController : ComputerModule
	{
		public int WaypointIndex = -1;
		public List<MechJebWaypoint> Waypoints = new List<MechJebWaypoint>();
		public MuMech.MovingAverage etaSpeed = new MovingAverage(50);

		// TODO: add loop control in WaypointWindow?
		public bool LoopWaypoints = false;


		[ToggleInfoItem("#MechJeb_ControlHeading", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Heading control
		public bool ControlHeading;

		[EditableInfoItem("#MechJeb_Heading", InfoItem.Category.Rover, width = 40), Persistent(pass = (int)Pass.Local)]//Heading
		public EditableDouble heading = 0;

		[ValueInfoItem("#MechJeb_Headingerror", InfoItem.Category.Rover, format = "F1", units = "º")]//Heading error
		public double headingErr;

		[ToggleInfoItem("#MechJeb_ControlSpeed", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Speed control
		public bool ControlSpeed = false;

		[EditableInfoItem("#MechJeb_Speed", InfoItem.Category.Rover, width = 40), Persistent(pass = (int)Pass.Local)]//Speed
		public EditableDouble speed = 10;

		[ValueInfoItem("#MechJeb_Speederror", InfoItem.Category.Rover, format = ValueInfoItem.SI, units = "m/s")]//Speed error
		public double speedErr;

		[ValueInfoItem("#MechJeb_Rover_label1", InfoItem.Category.Rover, format = ValueInfoItem.SI, units = "m/s")]//Target speed
		public double tgtSpeed;

		[ToggleInfoItem("#MechJeb_BrakeOnEject", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Brake on Pilot Eject
		public bool BrakeOnEject = false;

		[ToggleInfoItem("#MechJeb_BrakeOnEnergyDepletion", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Brake on Energy Depletion
		public bool BrakeOnEnergyDepletion = false;

		// TODO: "warp to daylight" really is "warp til recharged"
		[ToggleInfoItem("#MechJeb_WarpToDaylight", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Warp until Day if Depleted
		public bool WarpToDaylight = false;

		[ToggleInfoItem("#MechJeb_StabilityControl", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)]//Stability Control
		public bool StabilityControl = false;


		// TODO: no longer implemented
		[ToggleInfoItem("#MechJeb_LimitAcceleration", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local | (int)Pass.Type)]//Limit Acceleration
		public bool LimitAcceleration = false;

		[EditableInfoItem("#MechJeb_SafeTurnspeed", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Safe turnspeed
		public EditableDouble turnSpeed = 3;

		[EditableInfoItem("#MechJeb_TerrainLookAhead", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Terrain Look Ahead
		public EditableDouble terrainLookAhead = 1.0;

		[EditableInfoItem("#MechJeb_BrakeSpeedLimit", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Brake Speed Limit
		public EditableDouble brakeSpeedLimit = 0.7;

		[EditableInfoItem("#MechJeb_HeadingPIDP", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Heading PID P
		public EditableDouble hPIDp = 0.03; // 0.01

		[EditableInfoItem("#MechJeb_HeadingPIDI", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Heading PID I
		public EditableDouble hPIDi = 0.002; // 0.001

		[EditableInfoItem("#MechJeb_HeadingPIDD", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Heading PID D
		public EditableDouble hPIDd = 0.005;

		[EditableInfoItem("#MechJeb_SpeedPIDP", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Speed PID P
		public EditableDouble sPIDp = 2.0;

		[EditableInfoItem("#MechJeb_SpeedPIDI", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Speed PID I
		public EditableDouble sPIDi = 0.1;

		[EditableInfoItem("#MechJeb_SpeedPIDD", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Speed PID D
		public EditableDouble sPIDd = 0.001;

/*
		[ValueInfoItem("#MechJeb_SpeedIntAcc", InfoItem.Category.Rover, format = ValueInfoItem.SI, units = "m/s")]//Speed Int Acc
		public double speedIntAcc = 0;
*/

		[ValueInfoItem("#MechJeb_Traction", InfoItem.Category.Rover, units = "%")]//Traction
		public int traction = 0;

		[EditableInfoItem("#MechJeb_TractionBrakeLimit", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)]//Traction Brake Limit
		public EditableInt tractionLimit = 75;


		private PIDController headingPID;
		private PIDController speedPID;

		private CelestialBody lastBody = null;

		private bool waitingForDaylight = false;

		private double lastETA = 0;

		private float curSpeed;

		private List<ModuleWheelBase> wheelbases;

//		private float lastThrottle = 0;

//		private LineRenderer line;


		private static List<ModuleWheelBase> CountWheels(Vessel v)
		{
			// TODO: need to look into the ModuleWheelSteering ModuleWheelMotor ModuleWheelBrakes ModuleWheelBase ModuleWheelSuspension and see what they could bring

			// collect modules for ground contact check
			// TODO: what if a part has multiple ModuleWheelBase?
			// TODO: does every wheel has a ModuleWheelBase? check wheel part mods
			// TODO: check for brakes / motorized / broken wheels?
			// TODO: heading control could use a list of steerable wheels instead
			return v.Parts.Where(
				p => p.HasModule<ModuleWheelBase>()
				&& p.GetModule<ModuleWheelBase>().wheelType != WheelType.LEG
//				&& (p.GetModule<ModuleWheels.ModuleWheelSteering>()?.steeringEnabled == true || p.GetModule<ModuleWheels.ModuleWheelMotorSteering>()?.steeringEnabled == true)
			).Select(p => p.GetModule<ModuleWheelBase>()).ToList();
		}

		protected void OnVesselWasModified(Vessel v)
		{
			Profiler.BeginSample("OnVesselWasModified");
			if (v == vessel)
			{
				// TODO: re-use list?
				wheelbases = CountWheels(v);
print("MJRC: found " + wheelbases.Count + " wheels");
			}
			Profiler.EndSample();
		}

		private static float HeadingToPos(Vector3 fromPos, Vector3 toPos, CelestialBody body)
		{
			// thanks to Cilph who did most of this since I don't understand anything ~ BR2k
			bool dir = (MuUtils.ClampDegrees360(body.GetLongitude(toPos) - body.GetLongitude(fromPos)) > 180);
			Vector3 myPos = fromPos - body.transform.position;
			Vector3 tgtPos = toPos - fromPos;
			Vector3 north = body.transform.position + (body.transform.up * (float)body.Radius) - fromPos;
			return (dir ? -1 : 1) * Vector3.Angle(Vector3.ProjectOnPlane(north.normalized, myPos.normalized), Vector3.ProjectOnPlane(tgtPos.normalized, myPos.normalized));
		}

		private float HeadingToPos(Vector3 fromPos, Vector3 toPos)
		{
			return HeadingToPos(fromPos, toPos, mainBody);
		}

		// ramp down from speed to turnSpeed with increasing heading error
		private float TurningSpeed(double speed, double error)
		{
			return Mathf.Max((float)speed / Mathf.Max(Mathf.Abs((float)error) / 3f, 1f), (float)turnSpeed);
		}

		public override void OnStart(PartModule.StartState state)
		{
			Profiler.BeginSample("OnStart");

			base.OnStart(state);

			if (!HighLogic.LoadedSceneIsFlight)
			{
// TODO: could this module potentially be asked to Drive in other scenes?
print("MJRC: not my scene");
				return;
			}

			headingPID = new PIDController(hPIDp, hPIDi, hPIDd);
			speedPID = new PIDController(sPIDp, sPIDi, sPIDd);

			lastBody = mainBody;

//			MechJebRouteRenderer.NewLineRenderer(ref line);
//			line.enabled = false;

			GameEvents.onVesselWasModified.Add(OnVesselWasModified);

			Profiler.EndSample();
		}

		public override void OnDestroy()
		{
			Profiler.BeginSample("OnDestroy");

			GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

			base.OnDestroy();

			Profiler.EndSample();
		}

		public override void OnModuleDisabled()
		{
			Profiler.BeginSample("OnModuleDisabled");

			// TODO: clear out wheelbases, unregister GameEvent?

			if (core.attitude.users.Contains(this))
			{
//				line.enabled = false;
				core.attitude.attitudeDeactivate();
				core.attitude.users.Remove(this);
			}

			base.OnModuleDisabled();

			Profiler.EndSample();
		}

		public void CalculateTraction()
		{
			Profiler.BeginSample("CalculateTraction");

			if (wheelbases == null)
				OnVesselWasModified(vessel);

			// sum up percentage of wheels in ground contact
//			traction = wheelbases.Count > 0 ? wheelbases.Sum(p => p.isGrounded ? 100 : 0) / wheelbases.Count : 0;

			int result = 0;
			if (wheelbases.Count > 0)
			{
				for (int i = 0; i < wheelbases.Count; i++)
				{
					if (wheelbases[i].isGrounded)
						result += 100;
				}
				result /= wheelbases.Count;
			}
			traction = result;

			// clamp editable limit to 0-100
//			tractionLimit = Math.Min(Math.Max(tractionLimit, 0), 100);

			if (tractionLimit < 0)
				tractionLimit = 0;
			else if (tractionLimit > 100)
				tractionLimit = 100;

			Profiler.EndSample();
		}

		public override void Drive(FlightCtrlState s)
		{
//			Profiler.BeginSample("Drive");

			// TODO put the brake in when running out of power to prevent nighttime solar failures on hills, or atleast try to
			// TODO make distance calculation for 'reached' determination consider the rover and waypoint on sealevel to prevent height differences from messing it up -- should be done now?

			// clear waypoint list if CelestialBody changes
			// TODO: what about waypoint lists with varying bodies? wp.Body changes with ActiveVessel?
			// TODO: when do Vessel.mainBody and Vessel.orbit.referenceBody differ?
			// TODO: why was that also checked in FixedUpdate()?
			// TODO: only clears the list if the AP is enabled
			if (mainBody != lastBody)
			{
				WaypointIndex = -1;
				Waypoints.Clear();
				lastBody = mainBody;
			}

			bool brake = vessel.ActionGroups[KSPActionGroup.Brakes]; // keep brakes locked if they are

			// "forward" portion of surface velocity vector
//			curSpeed = Vector3d.Dot(vesselState.surfaceVelocity, vesselState.forward);
			curSpeed = Vector3.Dot(vesselState.surfaceVelocity, vesselState.forward);

//			speedIntAcc = speedPID.intAccum;

			// TODO: differentiate traction limits for driving, braking, steering?
			CalculateTraction();

			MechJebWaypoint wp = (WaypointIndex >= 0 && WaypointIndex < Waypoints.Count ? Waypoints[WaypointIndex] : null);

//			if (wp != null && wp.Body == mainBody)
			if (wp != null)
			{
				Profiler.BeginSample("WaypointAP");

				if (ControlHeading)
				{
					heading.val = Math.Round(HeadingToPos(vessel.CoM, wp.Position), 1);
				}

				if (ControlSpeed)
				{
					MechJebWaypoint nextWP = (WaypointIndex < Waypoints.Count - 1 ? Waypoints[WaypointIndex + 1] : (LoopWaypoints ? Waypoints[0] : null));
					float distance = Vector3.Distance(vessel.CoM, wp.Position);
					if (wp.Target != null) { distance += (float)(wp.Target.srfSpeed * curSpeed) / 2; }
					// var maxSpeed = (wp.MaxSpeed > 0 ? Mathf.Min(speed, wp.MaxSpeed) : speed); // use waypoints maxSpeed if set and smaller than set the speed or just stick with the set speed
					float maxSpeed = (wp.MaxSpeed > 0 ? wp.MaxSpeed : (float)speed); // speed used to go towards the waypoint, using the waypoints maxSpeed if set or just stick with the set speed
					float minSpeed = (wp.MinSpeed > 0 ? wp.MinSpeed :
								   (nextWP != null ? TurningSpeed((nextWP.MaxSpeed > 0 ? nextWP.MaxSpeed : (float)speed), heading - HeadingToPos(wp.Position, nextWP.Position)) :
								   (distance - wp.Radius > 50 ? (float)turnSpeed : 1)));
					minSpeed = (wp.Quicksave ? 1 : minSpeed);
					// ^ speed used to go through the waypoint, using half the set speed or maxSpeed as minSpeed for routing waypoints (all except the last)
					float newSpeed = Mathf.Min(maxSpeed, Mathf.Max((distance - wp.Radius) / curSpeed, minSpeed)); // brake when getting closer
					newSpeed = (newSpeed > turnSpeed ? TurningSpeed(newSpeed, headingErr) : newSpeed); // reduce speed when turning a lot
					float radius = Mathf.Max(wp.Radius, 10f);
					if (distance < radius)
					{
						if (WaypointIndex + 1 >= Waypoints.Count) // last waypoint
						{
							newSpeed = new [] { newSpeed, (distance < radius * 0.8 ? 0 : 1) }.Min();
							// ^ limit speed so it'll only go from 1m/s to full stop when braking to prevent accidents on moons
							if (LoopWaypoints)
							{
								WaypointIndex = 0;
							}
							else
							{
								newSpeed = 0;
								brake = true;
								if (curSpeed < brakeSpeedLimit)
								{
									if (wp.Quicksave)
									{
										//if (s.mainThrottle > 0) { s.mainThrottle = 0; }
										if (FlightGlobals.ClearToSave() == ClearToSaveStatus.CLEAR)
										{
											WaypointIndex = -1;
											ControlHeading = ControlSpeed = false;
											QuickSaveLoad.QuickSave();
										}
									}
									else
									{
										WaypointIndex = -1;
										ControlHeading = ControlSpeed = false;
									}
								}
							}
						}
						else
						{
							if (wp.Quicksave)
							{
								//if (s.mainThrottle > 0) { s.mainThrottle = 0; }
								newSpeed = 0;
								if (curSpeed < brakeSpeedLimit)
								{
									if (FlightGlobals.ClearToSave() == ClearToSaveStatus.CLEAR)
									{
										WaypointIndex++;
										QuickSaveLoad.QuickSave();
									}
								}
							}
							else
							{
								WaypointIndex++;
							}
						}
					}
					brake = brake || ((s.wheelThrottle == 0 || !vessel.isActiveVessel) && curSpeed < brakeSpeedLimit && newSpeed < brakeSpeedLimit);
					// ^ brake if needed to prevent rolling, hopefully
					tgtSpeed = (newSpeed >= 0 ? newSpeed : 0);
				}

				Profiler.EndSample();
			}

			if (ControlHeading)
			{
				Profiler.BeginSample("ControlHeading");

				headingPID.intAccum = Mathf.Clamp((float)headingPID.intAccum, -1f, 1f);

				headingErr = MuUtils.ClampDegrees180(vesselState.rotationVesselSurface.eulerAngles.y - heading);

				if (s.wheelSteer == s.wheelSteerTrim || !vessel.isActiveVessel)
				{
					// turnSpeed needs to be higher than curSpeed or it will never steer as much as it could even at 0.2m/s above it
					float limit = (Mathf.Abs(curSpeed) > turnSpeed ? Mathf.Clamp((float)((turnSpeed + 6) / (curSpeed*curSpeed)), 0.1f, 1f) : 1f);

					// double act = headingPID.Compute(headingErr * headingErr / 10 * Mathf.Sign(headingErr));
					float act = (float)headingPID.Compute(headingErr);

					// prevents it from flying above a waypoint and landing with steering at max while still going fast
					if (traction >= tractionLimit) {
						s.wheelSteer = Mathf.Clamp(act, -limit, limit);
					}
				}

				Profiler.EndSample();
			}

			// Brake if there is no controller (Pilot eject from seat)
			if (BrakeOnEject && vessel.GetReferenceTransformPart() == null)
			{
				s.wheelThrottle = 0;
				brake = true;
			}
			else if (ControlSpeed)
			{
				Profiler.BeginSample("ControlSpeed");

				speedPID.intAccum = Mathf.Clamp((float)speedPID.intAccum, -5f, 5f);

				speedErr = (WaypointIndex == -1 ? (double)speed : tgtSpeed) - curSpeed;

				if (s.wheelThrottle == s.wheelThrottleTrim || !vessel.isActiveVessel)
				{
					float act = (float)speedPID.Compute(speedErr);

					s.wheelThrottle = Mathf.Clamp(act, -1f, 1f);
					// s.wheelThrottle = (!LimitAcceleration ? Mathf.Clamp(act, -1, 1) : // I think I'm using these ( ? : ) a bit too much
						// (traction == 0 ? 0 : (act < 0 ? Mathf.Clamp(act, -1f, 1f) : (lastThrottle + Mathf.Clamp(act - lastThrottle, -0.01f, 0.01f)) * (traction < tractionLimit ? -1 : 1))));

					if (curSpeed < 0 & s.wheelThrottle < 0) { s.wheelThrottle = 0; } // don't go backwards
					if (Mathf.Sign(act) + Mathf.Sign(s.wheelThrottle) == 0) { s.wheelThrottle = Mathf.Clamp(act, -1f, 1f); }
					if (speedErr < -1 && StabilityControl && Mathf.Sign(s.wheelThrottle) + Mathf.Sign(curSpeed) == 0) { // StabilityControl && traction > 50 &&
						brake = true;
					}

//					lastThrottle = Mathf.Clamp(s.wheelThrottle, -1, 1);
				}

				Profiler.EndSample();
			}

			if (StabilityControl)
			{
				Profiler.BeginSample("StabilityControl");

				if (!core.attitude.users.Contains(this))
				{
					core.attitude.users.Add(this);
//					line.enabled = true;
				}

				// TODO: maybe tap into KER's impact predictor if available?
				RaycastHit hit;
				Physics.Raycast(vessel.CoM + vesselState.surfaceVelocity * terrainLookAhead + vesselState.up * 100, -vesselState.up, out hit, 500, 1 << 15, QueryTriggerInteraction.Ignore);
				Vector3 norm = hit.normal;

				Vector3 fwd = (Vector3)(traction > 0 ?
					vesselState.forward * 4f - vessel.transform.right * s.wheelSteer * Mathf.Sign(curSpeed) :
					vesselState.surfaceVelocity); // in the air so follow velocity
				Vector3.OrthoNormalize(ref norm, ref fwd);
				var quat = Quaternion.LookRotation(fwd, norm);

//				line.SetPosition(0, vessel.CoM);
//				line.SetPosition(1, vessel.CoM + norm * 5);
//				float scale = Vector3.Distance(FlightCamera.fetch.mainCamera.transform.position, vessel.CoM) / 900f;
//				line.SetWidth(0, scale + 0.1f);

				if (vesselState.torqueAvailable.sqrMagnitude > 0)
					core.attitude.attitudeTo(quat, AttitudeReference.INERTIAL, this);

				Profiler.EndSample();
			}

			// TODO: collect a list of batteries and panels and update in OnVesselWasModified?
			// TODO: identify key points in expected behaviour
			if (BrakeOnEnergyDepletion)
			{
				Profiler.BeginSample("BrakeOnEnergyDepletion");

				var batteries = vessel.Parts.FindAll(p => p.Resources.Contains(PartResourceLibrary.ElectricityHashcode) && p.Resources.Get(PartResourceLibrary.ElectricityHashcode).flowState);
				var energyLeft = batteries.Sum(p => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).amount) / batteries.Sum(p => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).maxAmount);
//				var openSolars = vessel.mainBody.atmosphere && // true if in atmosphere and there are breakable solarpanels that aren't broken nor retracted
				var openSolars = vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll(p => p.isBreakable && p.deployState != ModuleDeployablePart.DeployState.BROKEN && p.deployState != ModuleDeployablePart.DeployState.RETRACTED).Count > 0;

				if (openSolars && energyLeft > 0.99)
				{
					vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll(p => p.isBreakable &&
																							 p.deployState == ModuleDeployablePart.DeployState.EXTENDED).ForEach(p => p.Retract());
				}

				if (energyLeft < 0.05 && Mathf.Sign(s.wheelThrottle) + Mathf.Sign(curSpeed) != 0)
				{
					// save remaining energy by not using it for acceleration
					s.wheelThrottle = 0;
				}

//				if (openSolars || energyLeft < 0.03) { tgtSpeed = 0; }

//				if (curSpeed < brakeSpeedLimit && (energyLeft < 0.05 || openSolars))
				if (curSpeed < brakeSpeedLimit && energyLeft < 0.05)
				{
					s.wheelThrottle = 0;
					brake = true;
					vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll(p => p.isBreakable &&
																							 p.deployState == ModuleDeployablePart.DeployState.RETRACTED).ForEach(p => p.Extend());
				}

//					vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll(p => p.deployState == ModuleDeployablePart.DeployState.EXTENDED).Count > 0)
				if (curSpeed < 0.1 && energyLeft < 0.05 && !waitingForDaylight && openSolars)
				{
					waitingForDaylight = true;
				}

				Profiler.EndSample();
			}

			if (s.wheelThrottle != 0 && (Mathf.Sign(s.wheelThrottle) + Mathf.Sign(curSpeed) != 0 || curSpeed < 1f))
			{
				brake = false; // the AP or user want to drive into the direction of momentum so release the brake
			}

			if (vessel.isActiveVessel)
			{
				if (GameSettings.BRAKES.GetKeyUp())
				{
					brake = false; // release the brakes if the user lets go of them
				}
				if (GameSettings.BRAKES.GetKey())
				{
					brake = true; // brake if the user brakes and we aren't about to flip
				}
			}

			vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, brake && (StabilityControl && (ControlHeading || ControlSpeed) ? traction >= tractionLimit : true));
			// only let go of the brake when losing traction if the AP is driving, otherwise assume the player knows when to let go of it
			// also to not constantly turn off the parking brake from going over a small bump
			if (brake && curSpeed < 0.1)
			{
				s.wheelThrottle = 0;
			}

//			Profiler.EndSample();
		}

		public override void OnFixedUpdate()
		{
if (!HighLogic.LoadedSceneIsFlight) { print("MJRC: no fixedupdate"); return; }

//			Profiler.BeginSample("OnFixedUpdate");

//			if (lastBody != mainBody) { lastBody = mainBody; }

			if (!core.GetComputerModule<MechJebModuleWaypointWindow>().enabled)
			{
				Waypoints.ForEach(wp => wp.Update()); // update waypoints unless the waypoint window is (hopefully) doing that already
			}

			// TODO: why set those here and not in Drive()?
			headingPID.Kp = hPIDp;
			headingPID.Ki = hPIDi;
			headingPID.Kd = hPIDd;
			speedPID.Kp = sPIDp;
			speedPID.Ki = sPIDi;
			speedPID.Kd = sPIDd;

			if (lastETA + 0.2 < DateTime.Now.TimeOfDay.TotalSeconds)
			{
				etaSpeed.value = curSpeed;
				lastETA = DateTime.Now.TimeOfDay.TotalSeconds;
			}

			// TODO: what does RoverWindow.OnUpdate() do, anyway?
/*
			if (!core.GetComputerModule<MechJebModuleRoverWindow>().enabled)
			{
				core.GetComputerModule<MechJebModuleRoverWindow>().OnUpdate(); // update users for Stability Control, Brake on Eject and Brake on Energy Depletion
			}
*/
//			Profiler.EndSample();
		}

		public override void OnUpdate()
		{
if (!HighLogic.LoadedSceneIsFlight) { print("MJRC: no update"); return; }

//			Profiler.BeginSample("OnUpdate");

			if (WarpToDaylight && waitingForDaylight && vessel.isActiveVessel)
			{
				Profiler.BeginSample("BrakeOnEnergyDepletion");

				var batteries = vessel.Parts.FindAll(p => p.Resources.Contains(PartResourceLibrary.ElectricityHashcode) && p.Resources.Get(PartResourceLibrary.ElectricityHashcode).flowState);
				var energyLeft = batteries.Sum(p => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).amount) / batteries.Sum(p => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).maxAmount);

				// batteries full or no solar panels extended: deactivate warp
				if (energyLeft > 0.99 || vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll(p => p.deployState == ModuleDeployablePart.DeployState.EXTENDED).Count == 0)
				{
					waitingForDaylight = false;
					core.warp.MinimumWarp(false);
				}
				else
				{
					core.warp.WarpRegularAtRate(energyLeft < 0.9 ? 1000 : 50);
				}

				Profiler.EndSample();
			}
			else if (!WarpToDaylight && waitingForDaylight)
			{
				waitingForDaylight = false;
			}

			if (!core.GetComputerModule<MechJebModuleRoverWindow>().enabled)
			{
				core.GetComputerModule<MechJebModuleRoverWindow>().OnUpdate(); // update users for Stability Control, Brake on Eject and Brake on Energy Depletion
			}

			if (!StabilityControl && core.attitude.users.Contains(this))
			{
				core.attitude.attitudeDeactivate();
				core.attitude.users.Remove(this);
			}

//			Profiler.EndSample();
		}

		public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
		{
			Profiler.BeginSample("OnLoad");

			base.OnLoad(local, type, global);

			if (local != null)
			{
				var wps = local.GetNode("Waypoints");
				if (wps != null && wps.HasNode("Waypoint"))
				{
					WaypointIndex = -1;
					Waypoints.Clear();

					foreach (ConfigNode cn in wps.GetNodes("Waypoint"))
						Waypoints.Add(new MechJebWaypoint(cn));

//					Array.ForEach(wps.GetNodes("Waypoint"), cn => Waypoints.Add(new MechJebWaypoint(cn)));

					int.TryParse(wps.GetValue("Index"), out WaypointIndex);
					if (WaypointIndex >= Waypoints.Count)
						WaypointIndex = Waypoints.Count - 1;
				}
			}

			Profiler.EndSample();
		}

		public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
		{
			Profiler.BeginSample("OnSave");

			base.OnSave(local, type, global);

			if (local != null)
			{
				if (local.HasNode("Waypoints"))
					local.RemoveNode("Waypoints");

				if (Waypoints.Count > 0) {
					ConfigNode cn = new ConfigNode("Waypoints");

					cn.AddValue("Index", WaypointIndex);
					Waypoints.ForEach(wp => cn.AddNode(wp.ToConfigNode()));

					local.AddNode(cn);
				}
			}

			Profiler.EndSample();
		}

		public MechJebModuleRoverController(MechJebCore core) : base(core) { }
	}
}
