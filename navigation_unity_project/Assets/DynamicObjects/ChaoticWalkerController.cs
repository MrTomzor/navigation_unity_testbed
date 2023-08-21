using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ChaoticWalkerController : MonoBehaviour
{
  public NavMeshSurface surface;
  public GameObject managerObject;

  public float samplingRadius = 50;
  public float reachingDist = 2;
  bool hasGoal = false;
  public Vector3 goalPosition;
  public float thinkingPeriod = 0.5f;
  private float thinkTime = 0;

  private NavMeshAgent agentComponent;
  private float expectedMovementChange;
  private float reanchorDist;
  public float goalAbandonTimeAnchor = 5;
  public float goalAbandonTimeTotal = 20;
  private float goalGoingTime = 0;
  private float reanchorTime = 0;

  Vector3 lastAnchorPos;

  // Start is called before the first frame update
  void Start()
  {
    agentComponent = GetComponent<NavMeshAgent>();
    reanchorDist = agentComponent.speed;
    thinkTime = thinkingPeriod;
  }

  // Update is called once per frame
  void Update()
  {
    thinkTime += Time.deltaTime;
    if(thinkTime > thinkingPeriod){
      thinkTime -= thinkingPeriod;

      if(!hasGoal){
        Vector3 randomDirection = Random.insideUnitSphere * samplingRadius;
        randomDirection += managerObject.transform.position;

        NavMeshHit hit;
        NavMesh.SamplePosition(randomDirection, out hit, samplingRadius*10, 1);

        goalPosition = hit.position;
        hasGoal = true;
        goalGoingTime  = 0;
        reanchorTime  = 0;

        lastAnchorPos = transform.position;
      }
      if(hasGoal){
        goalGoingTime += Time.deltaTime;
        if(goalGoingTime > goalAbandonTimeTotal || reanchorTime > goalAbandonTimeAnchor){
          hasGoal = false;
        }

        if(Vector3.Distance(lastAnchorPos, transform.position) > reanchorDist){
          lastAnchorPos = transform.position;
          reanchorTime = 0;
        }

        agentComponent.SetDestination(goalPosition);
        if(Vector3.Distance(goalPosition, transform.position) < reachingDist){
          hasGoal = false;
        }
      }
    }
  }
}
