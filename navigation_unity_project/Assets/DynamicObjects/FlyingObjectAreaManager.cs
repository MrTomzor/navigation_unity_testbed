using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FlyingObjectAreaManager : MonoBehaviour
{
    public List<GameObject> spawnPrefabs;
    public int defaultNumObjectsToSpawn = 10;
    public float numerosity = 1;

    // Start is called before the first frame update
    void Start()
    {
      RespawnObjects();
    }

    // Update is called once per frame
    void Update()
    {
    }

    public Vector3 samplePosition(){
      float x = Random.Range(-1f,1f);
      float y = Random.Range(-1f,1f);
      float z = Random.Range(-1f,1f);
      Vector3 res = new Vector3(x,y,z) * 0.5f;
      res = transform.TransformPoint(res);
      return res;
    }

    public void RespawnObjects(){
      foreach (Transform child in transform) {
           Destroy(child.gameObject);
       }

      int numObjectsToSpawn = (int)(defaultNumObjectsToSpawn  * numerosity);

      for(int i = 0; i < numObjectsToSpawn; i++){
        GameObject spawnres = Instantiate(spawnPrefabs[Random.Range(0, spawnPrefabs.Count)], samplePosition(), transform.rotation);
        spawnres.transform.parent = gameObject.transform;
        spawnres.GetComponent<ChaoticFlightObjectController>().managerObject = gameObject;
      }
    }
}
