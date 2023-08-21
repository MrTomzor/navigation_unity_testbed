using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChildrenSpawnRandomizer : MonoBehaviour
{
  public double simpleRandomizationDeleteRate = 0.5;

  // Start is called before the first frame update
  void Start()
  {

  }

  // Update is called once per frame
  void Update()
  {

  }

  public void Randomize(){
    Transform parent_tr = GetComponent<Transform>();
    foreach(Transform child in parent_tr){
      if(Random.Range(0, 1f) < simpleRandomizationDeleteRate){
        child.gameObject.SetActive(false);
      }
      else{
        child.gameObject.SetActive(true);
      }
    }
  }
}
