using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class NavAreaManager : MonoBehaviour
{
    public int defaultNumObjectsToSpawn = 10;
    public float numerosity = 1;
    public NavMeshSurface surface;
    public float repeatPeriod = 0.2f;

    public float samplingRadius = 50f;

    public List<GameObject> spawnPrefabs;
    // Start is called before the first frame update
    void Start()
    {
      surface = GetComponent<NavMeshSurface>();
        
      InvokeRepeating("UpdateMesh", 0, repeatPeriod); 
      RespawnObjects();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void RespawnObjects(){
      if(surface == null){
        surface = GetComponent<NavMeshSurface>();
      }
      surface.BuildNavMesh();

      foreach (Transform child in transform) {
           Destroy(child.gameObject);
       }

      int numObjectsToSpawn = (int)(defaultNumObjectsToSpawn  * numerosity);

      for(int i = 0; i < numObjectsToSpawn; i++){
        Vector3 randomDirection = Random.insideUnitSphere * samplingRadius;
        randomDirection += transform.position;

        NavMeshHit hit;
        NavMesh.SamplePosition(randomDirection, out hit, samplingRadius*10, 1);

        GameObject spawnres = Instantiate(spawnPrefabs[Random.Range(0, spawnPrefabs.Count)], hit.position, transform.rotation);
        spawnres.transform.parent = gameObject.transform;
        spawnres.GetComponent<ChaoticWalkerController>().managerObject = gameObject;
        spawnres.GetComponent<ChaoticWalkerController>().surface = surface;
      }
    }

    void UpdateMesh(){
      surface.BuildNavMesh();
    }
}
