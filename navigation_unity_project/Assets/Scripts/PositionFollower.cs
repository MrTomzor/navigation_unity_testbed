using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PositionFollower : MonoBehaviour
{
    public GameObject target;
    public string search_tag = "robot";
    public bool find_with_tag = true;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
      if(target == null){
        var to = GameObject.FindGameObjectsWithTag(search_tag);
        if(to.Length > 0){
          target = to[0];
        }
      }
      else{
        transform.position = target.transform.position;
      }
    }
}
