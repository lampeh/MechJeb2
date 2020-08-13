using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Profiling;

// TODO: floats vs double vs int, Vector3 vs Vector3d, Mathf vs Math
// TODO: order - Vector * this * that vs this * that * Vector
// TODO: onFixedUpdate vs onUpdate
// TODO: check memory fragmentation
// TODO: compare profiler results

namespace MuMech
{
	public class MechJebModuleRoverController : ComputerModule
	{
		// TODO: add loop control in WaypointWindow?
		public bool LoopWaypoints = false;

		public int WaypointIndex = -1;
		public List<MechJebWaypoint> Waypoints = new List<MechJebWaypoint>();
		public MuMech.MovingAverage etaSpeed = new MovingAverage(50); // TODO: maybe rename avgSpeed?


		[ToggleInfoItem("#MechJeb_ControlHeading", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Heading control
		public bool ControlHeading;

		[EditableInfoItem("#MechJeb_Heading", InfoItem.Category.Rover, width = 40), Persistent(pass = (int)Pass.Local)] // Heading
		public EditableDouble heading = 0;

		[ValueInfoItem("#MechJeb_Headingerror", InfoItem.Category.Rover, format = "F1", units = "º")] // Heading error
		public double headingErr;

		[ToggleInfoItem("#MechJeb_ControlSpeed", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Speed control
		public bool ControlSpeed = false;

		[EditableInfoItem("#MechJeb_Speed", InfoItem.Category.Rover, width = 40), Persistent(pass = (int)Pass.Local)] // Speed
		public EditableDouble speed = 10;

		[ValueInfoItem("#MechJeb_Speederror", InfoItem.Category.Rover, format = ValueInfoItem.SI, units = "m/s")] // Speed error
		public double speedErr;

		[ValueInfoItem("#MechJeb_Rover_label1", InfoItem.Category.Rover, format = ValueInfoItem.SI, units = "m/s")] // Target speed
		public double tgtSpeed;

		[ToggleInfoItem("#MechJeb_StabilityControl", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Stability control
		public bool StabilityControl = false;

		[ToggleInfoItem("#MechJeb_BrakeOnEject", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Brake on Pilot Eject
		public bool BrakeOnEject = false;

		[ToggleInfoItem("#MechJeb_BrakeOnEnergyDepletion", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Brake on Energy Depletion
		public bool BrakeOnEnergyDepletion = false;

		// TODO: "warp to daylight" really is "warp til recharged"
		[ToggleInfoItem("#MechJeb_WarpToDaylight", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Warp until Day if Depleted
		public bool WarpToDaylight = false;
/*
		// TODO: brake if a wheel broke
		[ToggleInfoItem("#MechJeb_BrakeOnBreak", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local)] // Brake on Broken Wheels
		public bool BrakeOnBreak = false;
*/


/*
		// TODO: no longer implemented
		[ToggleInfoItem("#MechJeb_LimitAcceleration", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Local | (int)Pass.Type)] // Limit Acceleration
		public bool LimitAcceleration = false;
*/
		[EditableInfoItem("#MechJeb_SafeTurnspeed", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Safe turnspeed
		public EditableDouble turnSpeed = 3;

		[EditableInfoItem("#MechJeb_TerrainLookAhead", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Terrain Look Ahead
		public EditableDouble terrainLookAhead = 1.0;

		[EditableInfoItem("#MechJeb_BrakeSpeedLimit", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Brake Speed Limit
		public EditableDouble brakeSpeedLimit = 0.7;

		[EditableInfoItem("#MechJeb_HeadingPIDP", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Heading PID P
		public EditableDouble hPIDp = 0.03; // 0.01

		[EditableInfoItem("#MechJeb_HeadingPIDI", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Heading PID I
		public EditableDouble hPIDi = 0.002; // 0.001

		[EditableInfoItem("#MechJeb_HeadingPIDD", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Heading PID D
		public EditableDouble hPIDd = 0.005;

		[EditableInfoItem("#MechJeb_SpeedPIDP", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Speed PID P
		public EditableDouble sPIDp = 2.0;

		[EditableInfoItem("#MechJeb_SpeedPIDI", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Speed PID I
		public EditableDouble sPIDi = 0.1;

		[EditableInfoItem("#MechJeb_SpeedPIDD", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Speed PID D
		public EditableDouble sPIDd = 0.001;

		[ValueInfoItem("#MechJeb_Traction", InfoItem.Category.Rover, units = "%")] // Traction
		public int traction = 0;

		[EditableInfoItem("#MechJeb_TractionBrakeLimit", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // Traction Brake Limit
		public EditableInt tractionLimit = 75;


		private PIDController headingPID;
		private PIDController speedPID;

		private CelestialBody lastBody = null;

		private bool waitingForDaylight = false;

		private double lastETA = 0;

		private float curSpeed;
		private float curSpeedAbs;

		private List<ModuleWheelBase> wheelbases;

//		private LineRenderer line;


		// additional debug readouts

		// TODO: maybe floating point percentages are better in case of huge batteries
		[ValueInfoItem("#MechJeb_EnergyLeft", InfoItem.Category.Rover, units = "%")] // EC reserves
		public int energyLeft = 0;

/*
		// TODO: configurable limits
		[EditableInfoItem("#MechJeb_EnergyLimit", InfoItem.Category.Rover), Persistent(pass = (int)Pass.Type)] // EC limit
		public EditableInt energyLimit = 1;
*/

		[ValueInfoItem("Throttle", InfoItem.Category.Rover, units = "%")]
		public int lastThrottle = 0;

		[ValueInfoItem("Steering", InfoItem.Category.Rover, units = "%")]
		public int lastSteer = 0;


		// TODO: is this how it's done? why not ==?
		// TODO: Unity's Mathf.Sign returns 1 if val is 0, does that matter?
		private static bool SameSign(float val1, float val2)
		{
			return Mathf.Sign(val1) + Mathf.Sign(val2) != 0;
		}


		private static float HeadingToPos(Vector3 fromPos, Vector3 toPos, Transform origin)
		{
			// thanks to Cilph who did most of this since I don't understand anything ~ BR2k
			Vector3 up = fromPos - origin.position; // position relative to origin, "up" vector
			up.Normalize();

			// mark north and target directions on horizontal plane
			Vector3 north = Vector3.ProjectOnPlane(origin.up, up);
			Vector3 target = Vector3.ProjectOnPlane(toPos - fromPos, up); // no need to normalize

			// apply protractor
			return Vector3.SignedAngle(north, target, up);
		}

		private float HeadingToPos(Vector3 fromPos, Vector3 toPos)
		{
			return HeadingToPos(fromPos, toPos, mainBody.transform);
		}


		private static int EnergyLeft(Vessel v)
		{
			// TODO: maybe cache a list of resources or even a PartResourceList

			double amount = 0;
			double maxAmount = 0;

			v.GetConnectedResourceTotals(PartResourceLibrary.ElectricityHashcode, out amount, out maxAmount, true);

			return maxAmount > 0 ? (int)((amount * 100 / maxAmount) + 0.5) : 0;
		}

		private void EnergyLeft()
		{
			Profiler.BeginSample("EnergyLeft");
			energyLeft = EnergyLeft(vessel);
			Profiler.EndSample();
		}


		private static List<ModuleWheelBase> CountWheels(Vessel v)
		{
			// TODO: need to look into the ModuleWheelSteering ModuleWheelMotor ModuleWheelBrakes ModuleWheelBase ModuleWheelSuspension and see what they could bring

			// collect modules for ground contact check
			// TODO: what if a part has multiple ModuleWheelBase?
			// TODO: does every wheel has a ModuleWheelBase? check wheel part mods
			// TODO: MechJebModuleSpaceplaneAutopilot uses heading control - check landing gear parts
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
				wheelbases = null;
			}
			Profiler.EndSample();
		}

		public void CalculateTraction()
		{
			Profiler.BeginSample("CalculateTraction");

			if (wheelbases == null)
				wheelbases = CountWheels(vessel);

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

			// TODO: set .val to update editor?
			if (tractionLimit < 0)
				tractionLimit = 0;
			else if (tractionLimit > 100)
				tractionLimit = 100;

			Profiler.EndSample();
		}

		// ramp down from speed to turnSpeed with increasing heading error
		// TODO: only used by waypoint AP, not speed or heading control. should it?
		// TODO: consider traction of steerable wheels?
		// TODO: m/s / ° / 3?
		private float TurningSpeed(double speed, double error)
		{
			return Mathf.Max((float)speed / Mathf.Max(Mathf.Abs((float)error) / 3f, 1f), (float)turnSpeed);
		}


		public override void OnStart(PartModule.StartState state)
		{
			Profiler.BeginSample("OnStart");

			base.OnStart(state);

			// TODO: could this module potentially be asked to Drive in other scenes?
			if (!HighLogic.LoadedSceneIsFlight)
				return;

			headingPID = new PIDController(hPIDp, hPIDi, hPIDd);
			speedPID = new PIDController(sPIDp, sPIDi, sPIDd);

			lastBody = mainBody;

//			MechJebRouteRenderer.NewLineRenderer(ref line);
//			line.enabled = false;

//			GameEvents.onVesselWasModified.Add(OnVesselWasModified);

			Profiler.EndSample();
		}

		public override void OnDestroy()
		{
			Profiler.BeginSample("OnDestroy");

			GameEvents.onVesselWasModified.Remove(OnVesselWasModified);

			base.OnDestroy();

			Profiler.EndSample();
		}

		public override void OnModuleEnabled()
		{
			Profiler.BeginSample("OnModuleEnabled");

			base.OnModuleEnabled();

			GameEvents.onVesselWasModified.Add(OnVesselWasModified);

			Profiler.EndSample();
		}

		public override void OnModuleDisabled()
		{
			Profiler.BeginSample("OnModuleDisabled");

			GameEvents.onVesselWasModified.Remove(OnVesselWasModified);
			wheelbases = null;

			if (core.attitude.users.Contains(this))
			{
//				line.enabled = false;
				core.attitude.attitudeDeactivate();
				core.attitude.users.Remove(this);
			}

			base.OnModuleDisabled();

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
			curSpeed = Vector3.Dot(vesselState.surfaceVelocity, vesselState.forward);

			curSpeedAbs = Mathf.Abs(curSpeed);
			float curSpeedSign = Mathf.Sign(curSpeed);

			// TODO: differentiate traction limits for driving, braking, steering?
			CalculateTraction();

			MechJebWaypoint wp = (WaypointIndex >= 0 && WaypointIndex < Waypoints.Count ? Waypoints[WaypointIndex] : null);

//			if (wp != null && wp.Body == mainBody)
			if (wp != null)
			{
				Profiler.BeginSample("WaypointAP");

				if (ControlHeading)
				{
					// TODO: maybe avoid obstacles?
					heading.val = Math.Round(HeadingToPos(vessel.CoM, wp.Position), 1);
				}

				if (ControlSpeed)
				{
					MechJebWaypoint nextWP = (WaypointIndex < Waypoints.Count - 1 ? Waypoints[WaypointIndex + 1] : (LoopWaypoints ? Waypoints[0] : null));
					float distance = Vector3.Distance(vessel.CoM, wp.Position);

					if (wp.Target != null)
						distance += (float)(wp.Target.srfSpeed * curSpeed) / 2;

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
//							newSpeed = new [] { newSpeed, (distance < radius * 0.8 ? 0 : 1) }.Min();
							newSpeed = Mathf.Min(newSpeed, (distance < radius * 0.8f) ? 0f : 1f);
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
					brake = brake || ((s.wheelThrottle == s.wheelThrottleTrim || !vessel.isActiveVessel) && curSpeed < brakeSpeedLimit && newSpeed < brakeSpeedLimit);
					// ^ brake if needed to prevent rolling, hopefully
					tgtSpeed = (newSpeed >= 0 ? newSpeed : 0);
				}

				Profiler.EndSample();
			}

			if (ControlHeading)
			{
				Profiler.BeginSample("ControlHeading");

				headingErr = MuUtils.ClampDegrees180(vesselState.rotationVesselSurface.eulerAngles.y - heading);
// TODO: what should heading control do when driving backwards? keep forward heading or direction?
if (curSpeedSign < 0) { headingErr = MuUtils.ClampDegrees180(headingErr + 180); }

				if (s.wheelSteer == s.wheelSteerTrim || !vessel.isActiveVessel)
				{
					headingPID.intAccum = Mathf.Clamp((float)headingPID.intAccum, -1f, 1f);

					float act = (float)headingPID.Compute(headingErr) * curSpeedSign;

					// turnSpeed needs to be higher than curSpeed or it will never steer as much as it could even at 0.2m/s above it
					float limit = curSpeedAbs > turnSpeed ? Mathf.Clamp((float)((turnSpeed + 6) / (curSpeed*curSpeed)), 0.1f, 1f) : 1f;
//float limit = 1f;

					// prevents it from flying above a waypoint and landing with steering at max while still going fast
					if (traction >= tractionLimit)
						s.wheelSteer = Mathf.Clamp(act, -limit, limit);
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

				speedErr = (WaypointIndex == -1 ? (double)speed : tgtSpeed) - curSpeed;

				if (s.wheelThrottle == s.wheelThrottleTrim || !vessel.isActiveVessel)
				{
					speedPID.intAccum = Mathf.Clamp((float)speedPID.intAccum, -5f, 5f);

					s.wheelThrottle = Mathf.Clamp((float)speedPID.Compute(speedErr), -1f, 1f);

					if (Math.Abs(speedErr) > 1 && StabilityControl && !SameSign(s.wheelThrottle, curSpeed)) {
						brake = true;
					}
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

				if (vesselState.torqueAvailable.sqrMagnitude > 0)
				{
					// TODO: maybe tap into KER's impact predictor if available?
					// TODO: what does CelestialBody.GetImpactLatitudeAndLongitude() do?
					// TODO: consider actual attitude control delay instead of fixed terrainLookAhead?
					// TODO: resolve sign confusion - when going backwards, scan ground in velocity direction but point craft's forward in the opposite

					Vector3 fwd = traction > 0 ? // TODO: use traction limit? what about side slip?
						vesselState.forward * 4f - vessel.transform.right * s.wheelSteer * curSpeedSign : // TODO: why *4? momentum? why not *curSpeed?
						vesselState.surfaceVelocity * curSpeedSign; // in the air so follow velocity

					// cast a ray downwards from 100 units above the rover's expected position in terrainLookAhead seconds
					// assume a flat body at this scale
					RaycastHit hit;
					Vector3 norm;

					if (Physics.Raycast(vessel.CoM + fwd * curSpeedSign * (float)terrainLookAhead + vesselState.up * 100, -vesselState.up, out hit, 500, 1 << 15, QueryTriggerInteraction.Ignore))
						norm = hit.normal; // terrain slope
					else
						norm = vesselState.up; // no terrain in raydar range, just keep it level

// remove vertical component
//fwd = Vector3.ProjectOnPlane(fwd, vesselState.up);
fwd = Vector3.ProjectOnPlane(fwd, norm);
if (fwd == Vector3.zero) { fwd = vesselState.forward; }

					// point the craft forward, perpendicular to the surface
					Vector3.OrthoNormalize(ref norm, ref fwd);
					Quaternion quat = Quaternion.LookRotation(fwd, norm);

					// TODO: limit attitude controller roll limiter?
					core.attitude.attitudeTo(quat, AttitudeReference.INERTIAL, this);

//					line.SetPosition(0, vessel.CoM);
//					line.SetPosition(1, vessel.CoM + norm * 5);
//					float scale = Vector3.Distance(FlightCamera.fetch.mainCamera.transform.position, vessel.CoM) / 900f;
//					line.SetWidth(0, scale + 0.1f);
				}
else { core.attitude.attitudeDeactivate(); }

				Profiler.EndSample();
			}

			// TODO: collect a list of batteries and panels and update in OnVesselWasModified?
			// TODO: identify key points in expected behaviour
			// TODO: check Kopernicus solar panels, do they implement ModuleDeployableSolarPanel
			// TODO: consider sources other than ModuleDeployableSolarPanel (generators, converters)
			// TODO: low/high limits to reduce start/stop cycles, open/close panels only once

			if (BrakeOnEnergyDepletion)
			{
				Profiler.BeginSample("BrakeOnEnergyDepletion");

				// TODO: move to (Fixed)Update?
				EnergyLeft();

				// TODO: use MechJebModuleSolarPanelController?
				// TODO: why use only breakable panels?
				IEnumerable<ModuleDeployableSolarPanel> solarPanels = vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().Where(p =>
					p.isBreakable &&
					p.deployState != ModuleDeployablePart.DeployState.BROKEN
				);

				bool openSolars = solarPanels.Any(p => p.deployState == ModuleDeployablePart.DeployState.EXTENDED);

				// batteries full: retract panels
				// TODO: wait for completion?
				if (energyLeft >= 99 && openSolars)
				{
					foreach (var p in solarPanels)
						p.Retract();
				}

				// 5% left, going slow: stop and open solar panels
				if (energyLeft <= 5 && curSpeedAbs <= brakeSpeedLimit)
				{
					s.wheelThrottle = 0;
					brake = true;

					foreach (var p in solarPanels)
						p.Extend();
				}

				// energy low, going in the right direction: coast, save remaining energy by not using it for acceleration
				else if (energyLeft <= 5 && SameSign(s.wheelThrottle, curSpeed))
				{
					s.wheelThrottle = 0;
				}

				// TODO: what about energy low, *not* going in the right direction (i.e. rolling down the hill)

				// enerygy low, (almost) stopped, any open solar panels: ready for time warp
				if (energyLeft <= 5 && curSpeedAbs < 0.1f && !waitingForDaylight && openSolars)
				{
s.wheelThrottle = 0;
brake = true;
					waitingForDaylight = true;
				}

//				// TODO: overriding tgtSpeed here has no effect
//				if (openSolars || energyLeft <= 3) { tgtSpeed = 0; }
//				if (curSpeed < brakeSpeedLimit && (energyLeft <= 5 || openSolars))

				// energy very low, stability control enabled: brake while the reaction wheels still have power
				if (energyLeft <= 3 && StabilityControl)
				{
					s.wheelThrottle = 0;
					brake = true;
				}

				Profiler.EndSample();
			}

			if (brake && s.wheelThrottle != 0 && (SameSign(s.wheelThrottle, curSpeed) || curSpeedAbs < 1f))
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

			if (brake && curSpeedAbs < 0.1f)
			{
				s.wheelThrottle = 0;
			}

lastThrottle = (int)(s.wheelThrottle * 100);
lastSteer = (int)(s.wheelSteer * 100);

//			Profiler.EndSample();
		}

		public override void OnFixedUpdate()
		{
if (!HighLogic.LoadedSceneIsFlight) { print("MJRC: no fixedupdate"); return; }

//			Profiler.BeginSample("OnFixedUpdate");

			// TODO: why set those here and not in Drive()?
			headingPID.Kp = hPIDp;
			headingPID.Ki = hPIDi;
			headingPID.Kd = hPIDd;
			speedPID.Kp = sPIDp;
			speedPID.Ki = sPIDi;
			speedPID.Kd = sPIDd;

			// max. 5 samples per second -> 10-second average
			if (lastETA + 0.2 < DateTime.Now.TimeOfDay.TotalSeconds)
			{
				etaSpeed.value = curSpeedAbs;
				lastETA = DateTime.Now.TimeOfDay.TotalSeconds;
			}

			if (!StabilityControl && core.attitude.users.Contains(this))
			{
				core.attitude.attitudeDeactivate();
				core.attitude.users.Remove(this);
			}

			if (!core.GetComputerModule<MechJebModuleWaypointWindow>().enabled)
			{
				Waypoints.ForEach(wp => wp.Update()); // update waypoints unless the waypoint window is (hopefully) doing that already
			}

			// TODO: what does RoverWindow.OnUpdate() do, anyway?
			// it updates the menu highlight when another module activates the AP (SpaceplaneAutopilot)
			// and keeps the module running while the autopilot is active even if the other module unregisters
			// then shuts the controller down, e.g. after reaching the final waypoint

			ComputerModule controllerWindow = core.GetComputerModule<MechJebModuleRoverWindow>();
			if (!controllerWindow.enabled)
				controllerWindow.OnUpdate(); // update users for AP features

//			Profiler.EndSample();
		}

		public override void OnUpdate()
		{
if (!HighLogic.LoadedSceneIsFlight) { print("MJRC: no update"); return; }

//			Profiler.BeginSample("OnUpdate");

			if (WarpToDaylight && waitingForDaylight && vessel.isActiveVessel)
			{
				Profiler.BeginSample("BrakeOnEnergyDepletion");

				EnergyLeft();

				bool openSolars = vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().Where(p =>
					p.deployState == ModuleDeployablePart.DeployState.EXTENDED
				).Any();

				// batteries full or no solar panels extended: deactivate warp
				// TODO: prevents player from time-warping with closed/broken/no solar panels
				if (energyLeft >= 99 || !openSolars)
				{
					waitingForDaylight = false;
					core.warp.MinimumWarp(false);
				}
				else
				{
					// TODO: get out of phys warp first?
					// TODO: warp faster through the night
					core.warp.WarpRegularAtRate(energyLeft < 90 ? 1000 : 50);
				}

				Profiler.EndSample();
			}
			else if (!WarpToDaylight && waitingForDaylight)
			{
				waitingForDaylight = false;
			}

//			Profiler.EndSample();
		}

		public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
		{
//			Profiler.BeginSample("OnLoad");

			base.OnLoad(local, type, global);

			if (local != null)
			{
				ConfigNode wps = local.GetNode("Waypoints");

				if (wps != null && wps.HasNode("Waypoint"))
				{
					WaypointIndex = -1;
					Waypoints.Clear();

					foreach (ConfigNode cn in wps.GetNodes("Waypoint"))
						Waypoints.Add(new MechJebWaypoint(cn));

					int.TryParse(wps.GetValue("Index"), out WaypointIndex);
					if (WaypointIndex >= Waypoints.Count)
						WaypointIndex = Waypoints.Count - 1;
				}
			}

//			Profiler.EndSample();
		}

		public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
		{
//			Profiler.BeginSample("OnSave");

			base.OnSave(local, type, global);

			if (local != null)
			{
				if (local.HasNode("Waypoints"))
					local.RemoveNode("Waypoints");

				if (Waypoints.Count > 0)
				{
					ConfigNode cn = new ConfigNode("Waypoints");

					cn.AddValue("Index", WaypointIndex);
					Waypoints.ForEach(wp => cn.AddNode(wp.ToConfigNode()));

					local.AddNode(cn);
				}
			}

//			Profiler.EndSample();
		}

		public MechJebModuleRoverController(MechJebCore core) : base(core) { }
	}
}
