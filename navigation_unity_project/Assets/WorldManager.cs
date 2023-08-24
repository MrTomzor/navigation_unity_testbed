using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor;

using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.NavigationUnity;
using RosMessageTypes.Std;

using System.IO;
/* using Xunit.Abstractions; */
using YamlDotNet.RepresentationModel;
/* using YamlDotNet.Serialization; */
/* using YamlDotNet.Samples.Helpers; */


public class WorldManager : MonoBehaviour
{
  /* public WorldConfig config; */
  [SerializeField] public string _taskdefTopicName = "task_definition";
  [SerializeField] public string _robotSleepStateTopicName = "robot_sleep_state";
  public static string world_config_string;
  public static string world_name;
  public static string spawn_area;
  public static int spawn_area_point_index;

  public static bool scene_init_flag = false;
  private bool scene_load_flag = false;
  private bool scene_modify_flag = false;
  private bool do_respawning_flag = true;

  public Material skyboxMaterial;
  public Material fogMaterial;

  /* public static bool is_waiting_for_first_world_config; */

  public int level;
  public double timeElapsed;
  public string task = "explore";
  public string robotType = "generic_wheeled";
  public GameObject robotPrefab_GenericWheeled;
  public GameObject robotPrefab_GenericSpace;
  public GameObject missionOriginPrefab;

  public string trialType = "explore+kidnap+home"; // or guidedexplore+kidnap+gotox or guidedexplore+kidnap+gotox+gotox or explore+detectchanges
  /* SPECIFIC SETTINGS FOR EXPLOREKIDNAPHOME */
  public double exploreTime = 1200;
  public double homingTimeMultiplier = 1.5; //multiplies the lenght of the path on the mesh
  // or fuel?
  public int kidnapTimes = 10;


  public double taskMaxTime = 60;
  public double taskElapsedTime = 0;


  public ChildrenSpawnRandomizer[]  simpleRandomizationGroups;

  private bool robotAwake = false;
  public GameObject robotGameObject;

  /* RANDOMIZATION */
  public int sessionSeed = 0;

  /* GLOBAL LIGHT DYNAMICS */
  public bool enableDayNightSimulation = true;
  public bool enableDayNightColorChange = false;
  public GameObject sunlightObject;
  public float dayNightCycleSeconds = 120f;
  public float dayNightMaxIntensity = 1.2f;
  public float dayNightMinIntensity = 0.8f;
  public float dayTime = 0f;

  public Vector3 globalLightAngle = Vector3.zero;

  public bool enableDynamicLightEffects = true;

  /* ROBOT SPAWNING */
  private LabeledZoneStringPublisher[] spawnAreas;

  /* DYNAMIC OBJECTS */
  private NavAreaManager[] groundDynamicObjectSpawners;
  private FlyingObjectAreaManager[] nongroundDynamicObjectSpawners;
  public float dynamicGroundObjectNumerosity = 1;
  public float dynamicNongroundObjectNumerosity = 1;

  /* CLOUDS AND PARTICLES */
  private GameObject cloudsObject;
  public bool cloudsEnabled = true;
  private GameObject[] rainObjects;
  public bool rainEnabled;
  public float cloudsDensity = 0.8f;
  public float cloudsBaseRate = 300;
  public float cloudsBrightness = 1;

  public float cloudsParticleScale = 1;
  public Vector3 cloudsBaseParticleScale = Vector3.zero;
  public float cloudsAverageVelocityX = 5;
  public float cloudsAverageVelocityZ = 5;
  public bool distractingParticlesEnabled = true;
  private GameObject[] distractingParticlesObjects;

  /* ROS STUFF */
  private ROSConnection _ros;
  private const string awake_string = "awake";
  private const string asleep_string = "asleep";

  public double sleepstateSendRate = 1;
  public double sleepstateElapsedTime = 0;

  /* private readonly ITestOutputHelper output; */

  void ModifyWorldFromStaticConfig(){
    // Setup the input
    var input = new StringReader(world_config_string);

    // Load the stream
    var yaml = new YamlStream();
    yaml.Load(input);

    // Examine the stream
    var mapping =
      (YamlMappingNode)yaml.Documents[0].RootNode;

    foreach (var entry in mapping.Children)
    {
      /* output.WriteLine(((YamlScalarNode)entry.Key).Value); */
      var key = ((YamlScalarNode)entry.Key).Value;
      var val = ((YamlScalarNode)entry.Value).Value;
      Debug.Log(key);
      Debug.Log(val);

      /* TODO move this to loading from string */
      if(key == "session_seed"){
        sessionSeed  = int.Parse(val);
      }

      /* SPAWNING */
      else if(key == "spawn_area"){
        spawn_area = (val);
      }
      else if(key == "spawn_point_number"){
        spawn_area_point_index = int.Parse(val);
      }
      else if(key == "robot_type"){
        robotType = val;
      }

      /* LIGHT */
      else if(key == "global_light_cycle_seconds"){
        dayNightCycleSeconds = float.Parse(val);
      }
      else if(key == "global_light_cycle_min_light"){
        dayNightMinIntensity = float.Parse(val);
      }
      else if(key == "global_light_cycle_max_light"){
        dayNightMaxIntensity = float.Parse(val);
      }
      else if(key == "global_light_cycle_start"){
        dayTime = float.Parse(val);
      }
      else if(key == "global_light_cycle_enabled"){
        enableDayNightSimulation = bool.Parse(val);
      }
      else if(key == "global_light_cycle_min_light"){
        dayNightMinIntensity = float.Parse(val);
      }
      else if(key == "global_light_yaw"){
        /* dayNightMinIntensity = float.Parse(val); */
        globalLightAngle.y = float.Parse(val);
      }
      else if(key == "global_light_pitch"){
        /* dayNightMinIntensity = float.Parse(val); */
        globalLightAngle.x = float.Parse(val);
      }

      /* FOG */
      else if(key == "fog_enabled"){
        RenderSettings.fog = bool.Parse(val);
        if(RenderSettings.fog){
          RenderSettings.skybox = fogMaterial;
        }
        else{
          RenderSettings.skybox = skyboxMaterial;
        }
      }
      else if(key == "fog_density"){
        RenderSettings.fogDensity = float.Parse(val);
      }

      /* CLOUDS AND PARTICLES */
      else if(key == "clouds_enabled"){
        /* RenderSettings.fogDensity = float.Parse(val); */
        cloudsEnabled = bool.Parse(val);
        cloudsObject.SetActive(cloudsEnabled);
      }
      else if(key == "clouds_height"){
        Vector3 tmp = cloudsObject.transform.position;
        tmp.y = float.Parse(val);
        cloudsObject.transform.position = tmp;
      }
      else if(key == "clouds_density"){
        cloudsDensity = float.Parse(val);
        ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();
        var emission = ps.emission;
        emission.rateOverTime = cloudsBaseRate * cloudsDensity;
      }
      else if(key == "clouds_brightness"){
        cloudsBrightness = float.Parse(val);

        ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();
        var main = ps.main;
        main.startColor = new Color(cloudsBrightness * 255,cloudsBrightness * 255,cloudsBrightness * 255);
      }
      else if(key == "clouds_particle_scale"){
        cloudsParticleScale = float.Parse(val);

        ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();
        var main = ps.main;
        main.startSizeXMultiplier = cloudsBaseParticleScale.x * cloudsParticleScale;
        main.startSizeYMultiplier = cloudsBaseParticleScale.y * cloudsParticleScale;
        main.startSizeZMultiplier = cloudsBaseParticleScale.z * cloudsParticleScale;
      }
      else if(key == "clouds_average_velocity_x"){
        cloudsAverageVelocityX = float.Parse(val);

        ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();

        var velocityOverLifetime = ps.velocityOverLifetime;
        /* velocityOverLifetime.space = ParticleSystemSimulationSpace.World; */
        velocityOverLifetime.x = cloudsAverageVelocityX;
      }
      else if(key == "clouds_average_velocity_Z"){
        cloudsAverageVelocityZ = float.Parse(val);

        ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();

        var velocityOverLifetime = ps.velocityOverLifetime;
        /* velocityOverLifetime.space = ParticleSystemSimulationSpace.World; */
        velocityOverLifetime.z = cloudsAverageVelocityZ;
      }
      else if(key == "distracting_particles_enabled"){
        distractingParticlesEnabled = bool.Parse(val);
        foreach(GameObject o in distractingParticlesObjects){
          o.SetActive(distractingParticlesEnabled);
        }
      }
      else if(key == "rain_enabled"){
        rainEnabled = bool.Parse(val);
        foreach(GameObject o in rainObjects){
          o.SetActive(rainEnabled);
        }
      }

      /* DYNAMIC OBJECTS */
      else if(key == "dynamic_light_effects_enabled"){
        enableDynamicLightEffects = bool.Parse(val);
        LightFlickerEffect.globally_enabled = enableDynamicLightEffects;
      }
      else if(key == "dynamic_ground_objects_numerosity"){
        /* dayNightMinIntensity = float.Parse(val); */
        dynamicGroundObjectNumerosity = float.Parse(val);

        foreach (NavAreaManager m in groundDynamicObjectSpawners){
          m.numerosity = dynamicGroundObjectNumerosity;
          if(dynamicGroundObjectNumerosity == 0){
            m.gameObject.SetActive(false);
          }
          else{
            m.gameObject.SetActive(true);
          }
        }
      }
      else if(key == "dynamic_nonground_objects_numerosity"){
        dynamicNongroundObjectNumerosity = float.Parse(val);
        foreach (FlyingObjectAreaManager m in nongroundDynamicObjectSpawners){
          m.numerosity = dynamicNongroundObjectNumerosity;
          if(dynamicNongroundObjectNumerosity == 0){
            m.gameObject.SetActive(false);
          }
          else{
            m.gameObject.SetActive(true);
          }
        }
      }

      else if(key == "respawn_random_objects"){
        resetWorld();
      }

    }

    if(do_respawning_flag){
      Debug.Log("spawning robot");
      spawnRobot(spawn_area, spawn_area_point_index);
      /* TODO respawn other shit too */
    }

    bakeDayNightEffects();
  }

  // Start is called before the first frame update
  void Start()
  {
    /* INIT ROS */
    this._ros = ROSConnection.GetOrCreateInstance();
    this._ros.RegisterPublisher<NavigationTaskDefinitionMsg>(this._taskdefTopicName);
    this._ros.RegisterPublisher<StringMsg>(this._robotSleepStateTopicName);
    /* this._ros.Subscribe<StringMsg>("/world_config", HandleWorldConfigMsg); */
    this._ros.ImplementService<ResetWorldRequest, ResetWorldResponse>("/reset_world", handleWorldResetRequest);

    /* FIND STUFF */
    groundDynamicObjectSpawners = (NavAreaManager[]) GameObject.FindObjectsOfType (typeof(NavAreaManager));
    nongroundDynamicObjectSpawners = (FlyingObjectAreaManager[]) GameObject.FindObjectsOfType (typeof(FlyingObjectAreaManager));
    simpleRandomizationGroups = (ChildrenSpawnRandomizer[]) GameObject.FindObjectsOfType (typeof(ChildrenSpawnRandomizer));
    cloudsObject = GameObject.FindGameObjectsWithTag("Clouds")[0];
    rainObjects = GameObject.FindGameObjectsWithTag("Rain");
    spawnAreas = (LabeledZoneStringPublisher[]) GameObject.FindObjectsOfType (typeof(LabeledZoneStringPublisher));

    {
      ParticleSystem ps = cloudsObject.GetComponent<ParticleSystem>();
      var main = ps.main;
      cloudsBaseParticleScale =  new Vector3(main.startSizeXMultiplier, main.startSizeYMultiplier, main.startSizeZMultiplier);
    }

    distractingParticlesObjects = GameObject.FindGameObjectsWithTag("DistractingParticles");

    /* FIRST RESET */
    do_respawning_flag = true;
    resetWorld();
    robotAwake = true;

    /* SEND MSG TO WAKE UP ROBOT */
    sendRobotSleepState();
  }

  ResetWorldResponse handleWorldResetRequest(ResetWorldRequest req){
    Debug.Log("RECEIVED REQUEST TO RESET WORLD");
    ResetWorldResponse  resp = new ResetWorldResponse ();
    resp.success = true;

    Debug.Log("----setting received string as world config yaml string: ");
    Debug.Log(req.world_config);
    world_config_string = req.world_config;

    // Setup the input
    var input = new StringReader(req.world_config);

    // Load the stream
    var yaml = new YamlStream();
    yaml.Load(input);

    // Examine the stream
    var mapping =
      (YamlMappingNode)yaml.Documents[0].RootNode;


    foreach (var entry in mapping.Children)
    {
      /* output.WriteLine(((YamlScalarNode)entry.Key).Value); */
      var key = ((YamlScalarNode)entry.Key).Value;
      var val = ((YamlScalarNode)entry.Value).Value;
      Debug.Log(key);
      Debug.Log(val);

      if(key == "world_name"){
        /* SET LOAD SCENE FLAG (do in next update) */
        world_name = val;
        /* SET RETURN VALUE */
        /* RETURN TO COMPLETE SERVICE*/
      }
      if(key == "reload_scene"){
        if(bool.Parse(val)){
          Debug.Log("LOADING SCENE");
          scene_load_flag = true;
        }
        else{
scene_modify_flag  = true;
        }
      }

      /* TODO move this to loading from string */
      if(key == "session_seed"){
        sessionSeed  = int.Parse(val);
      }
      else if(key == "spawn_area"){
        spawn_area = (val);
      }
      else if(key == "spawn_point_number"){
        spawn_area_point_index = int.Parse(val);
      }

    }

    return resp;
  }

  void HandleWorldConfigMsg(StringMsg  msg)
  {
    /* Debug.Log("----received string: "); */
  }

  bool spawnRobot(string zonename, int spawnpoint_index){
    /* FIND ALL EXISTING ROBOTS AND SPAWNPOINT ORIGIN AND REMOVE */
    var last_spawnpoint_tf_object = GameObject.Find("MISSION_ORIGIN_TF");
    if(last_spawnpoint_tf_object != null){
      Destroy(last_spawnpoint_tf_object);
    }
    var existing_robots = GameObject.FindGameObjectsWithTag("Player");
    foreach(GameObject o in existing_robots){
      Destroy(o);
    }

    /* CHOOSE ROBOT TYPE */
    GameObject usedPrefab;
    if(robotType == "generic_wheeled"){
      usedPrefab = robotPrefab_GenericWheeled;
    }
    else if(robotType == "generic_space"){
      usedPrefab = robotPrefab_GenericSpace;
    }
    else{
      Debug.Log("ERROR! robot type " + robotType + " NOT IMPLEMENTED! To add a new robot, add it to the spawnRobot() function in WorldManager.cs");
      return false;
    }

    /* FIND AREA WHERE TO SPAWN ROBOT */
    foreach(LabeledZoneStringPublisher area in spawnAreas){
      if(area == null){
        continue;
      }
      if(area.areaName == zonename){
        Debug.Log("found spawn area, spawning");

        /* SPAWN ROBOT */
        GameObject robotObject = Instantiate(usedPrefab) as GameObject;
        var sptr = area.getSpawnpointTransform(spawnpoint_index);
        robotObject.transform.position = sptr.position;
        robotObject.transform.rotation = sptr.rotation;

        /* ALSO SPAWN TF PUBLISHER AT MISSION START POSE */
        GameObject tfObject = Instantiate(missionOriginPrefab) as GameObject;
        tfObject.transform.position = sptr.position;
        tfObject.transform.rotation = sptr.rotation;
        tfObject.GetComponent<ROSSingleTransformPublisher>().secondObject = robotObject;

        /* ALSO SET THE TARGET OBJECT FOR WORLD TRANSFORM PUBLISHER */
        var world_origin_pub = GameObject.Find("WORLD_ORIGIN_TF");
        if(world_origin_pub != null){
          world_origin_pub.GetComponent<ROSSingleTransformPublisher>().secondObject = robotObject;
        }
        else{
          Debug.Log("WARN! no world origin tf publisher!");
        }

        /* ALSO ADD THE CAMERA AND ROBOT LINK TO THE SIM UI MANAGER */
        /* var simmanager = GetComponent<SimUIManager>(); */
        /* simmanager.controlledRobot = robotObject; */
        /* simmanager.debugCamera = GameObject.FindGameObjectsWithTag("DebugCamera")[0]; */
        /* Debug.Log("DEBUGCAM NAME: " + simmanager.debugCamera.name + "PARENT NAME: " + simmanager.debugCamera.transform.parent.gameObject.name); */
        /* /1* Debug.Log("FOUND CAMERA? " + string()) *1/ */

          Debug.Log("SPAWNED ROBOT!");
        return true;
      }
    }

    Debug.Log("ERROR! spawn area " + zonename + " DOES NOT EXIST!");
    return false;
  }

  // Update is called once per frame
  void Update()
  {

    if(scene_load_flag){
      scene_init_flag  = true;
      scene_load_flag = false;
      do_respawning_flag  = true;
      SceneManager.LoadScene(world_name, LoadSceneMode.Single);
      return;
    }
    else if(scene_modify_flag  ){
      scene_init_flag  = true;
      scene_modify_flag = false;
      do_respawning_flag  = false;
    }

    if(scene_init_flag){
      ModifyWorldFromStaticConfig();
      scene_init_flag = false;
    }

    /* SIM DAY AND NIGHT LIGHT CHANGES */
    if(enableDayNightSimulation){
      dayTime += Time.deltaTime;
      if(dayTime > dayNightCycleSeconds){
        dayTime -= dayNightCycleSeconds;
      }
      bakeDayNightEffects();
    }


    /* PERIODICALLY SEND SLEEPSTATE AND TASKDEF */
    sleepstateElapsedTime += Time.deltaTime;
    if(sleepstateElapsedTime > 1f/sleepstateSendRate){
      /* Debug.Log("sending robot sleep state"); */
      sleepstateElapsedTime -= 1f/sleepstateSendRate;
      sendRobotSleepState();
      sendCurrentTaskDefinition();
    }
  }

  void bakeDayNightEffects(){
    float mod = Mathf.Cos(2f * Mathf.PI * (dayTime / dayNightCycleSeconds));
    sunlightObject.transform.eulerAngles = globalLightAngle;
    sunlightObject.GetComponent<Light>().intensity = dayNightMinIntensity + (dayNightMaxIntensity - dayNightMinIntensity) * mod;
  }

  void resetWorld(){

    /* RANDOMIZE SIMPLE OBJECTS */
    foreach (ChildrenSpawnRandomizer  r in simpleRandomizationGroups){
      r.Randomize();
    }

    /* SPAWN DYNAMIC OBJECTS */
    foreach (NavAreaManager m in groundDynamicObjectSpawners){
      m.numerosity = dynamicGroundObjectNumerosity;
      if(dynamicGroundObjectNumerosity == 0){
        m.gameObject.SetActive(false);
      }
      else{
        m.gameObject.SetActive(true);
        m.RespawnObjects();
      }
    }

    foreach (FlyingObjectAreaManager m in nongroundDynamicObjectSpawners){
      m.numerosity = dynamicNongroundObjectNumerosity;
      if(dynamicNongroundObjectNumerosity == 0){
        m.gameObject.SetActive(false);
      }
      else{
        m.gameObject.SetActive(true);
        m.RespawnObjects();
      }
    }

  }

  void sendCurrentTaskDefinition(){
    NavigationTaskDefinitionMsg msg = new NavigationTaskDefinitionMsg();
    msg.task_type = task;
    msg.task_seconds = (float)taskMaxTime;
    msg.task_elapsed_seconds = (float)taskElapsedTime;
    this._ros.Publish(this._taskdefTopicName, msg);
  }

  void sendRobotSleepState(){
    StringMsg msg = new StringMsg();
    if(robotAwake){
      msg.data = awake_string;
    }
    else{
      msg.data = asleep_string;
    }
    this._ros.Publish(this._robotSleepStateTopicName, msg);
  }
}
