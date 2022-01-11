#include <iostream>
#include <queue>
using namespace std;

/*
* V - S 내의 정점 중 연결선 길이가 제일 작은 도착지 정점 w를 찾아내고
그걸 S에 추가한 뒤 방금까지 w가 속해있었던 V - S의 모든 정점에 대해 연결선 길이 갱신.
(지금까지 밝혀진 w까지의 최단경로 + 배열에 w에 명시된 모든 정점까지 각각의 거리) 가 원래보다 작으면 그걸로 갱신.

w랑 연결되지 않은 정점도 무연결(inf) 표시가 있으니까 건드림.
*/

void dijkstra(int(*graph)[9], int start)
{
    bool S[9] = { 0 }; //0 ~ 9까지의 각 정점을 조사안해도 되는가(true) 해야되는가(false)를 구분. 이 배열하나로 S와 V-S를 나타냄.
    int D[9]; //시작점start로부터 최단 길이 저장 하는 곳. 최단길이만 필요할 때 경로가 아닌 길이만 저장하는 게 코드 상 편리. 여기서 0은 제일 큰 숫자 취급임.
    /*int P[9][8]; path : 0~8 까지의 정점으로 가는 최단 경로들 x 아무리 많은 정점을 지나도 한 경로 당 모든 정점 개수 - 시작정점 하나 까지가 최대임.
                 일단 최단 거리가 두 개 이상 있는 경우는 신경쓰지 않고 코드 짬.
                 사용법 : -1이 저장되있으면 지금까지 발견된 경로가 여기까지란 뜻이니 발견될때마다 계속 밀어내줘야 함.
                 잠만 그럼 굳이 이럴 게 아니라 큐를 쓰면 되는 거 아닌가?
                 */
    queue<int> P[9]; /*그래서 만든 큐. 일단 경로가 발견되는대로 순서대로 정점을 넣음.
                     
                     처음엔 시작점도 좀 건드려서 경로 초기화가 필요할 듯

                     아래 for(j)문 내에 두번째 for(i)문에서
                     D[i] > D[w] + graph[w][i] 가 성립하면 w라는 정점까지 가는 경로를 갖고와서
                     (이미 이 시점에서 w까지 가는 경로는 반드시 발견된 걸로 확신하고 가겠음)
                     그 마지막에 이 다음 i로 가세요를 추가하면 되고

                     만약 더 짧은 새로운 경로가 발견되면
                     원래 경로랑 신 경로가 어디서부터 갈라지는지 보...는 식으로 하면 안되는구나
                     그냥 기존경로 다 날려버리고 새로 씌우셈
                     
                     나중에 경로 출력 시
                     큐에 있는 거 다꺼내도 도착점에 도착하지 못하면 갈 수 있는 경로가 하나도 없다고 출력
                     (아니 잠만 애초에 경로 미발견 시 그 큐는 아예 비어있을거 같은데. 그럼 큐로 만들 필요가 있나?
                     암만 복잡해져도 그냥 배열로 만들고 첫 자리에 -1 저장하는 방식이면 간단할 거 같은데)
                     */

    S[start] = true;

    for (int i = 0; i < 9; ++i) {
        D[i] = graph[start][i]; //일단 냅다 시작점 기준으로 최단거리 잡기. [n][m]은 n으로부터 m까지의 거리임.
    }

    for (int j = 0; j < 9; ++j) { //정확히는 S의 모든 원소가 true가 될때까지만 반복인데 어차피 한번 할때마다 무조건 하나씩 true로 바뀌어서 그냥 9번 실행이랑 똑같음
        int w = start; //복잡하게 코드짜면 안그래도 되지만 코드 간소화 하느라 0이 제일 크다는 개념왜곡의 영향으로 D[w]가 0인 w로 계속 초기화 해줘야 하는데 그런 자리는 start 임.

        for (int i = 0; i < 9; ++i) { //w를 정하기
            if (D[i] != 0 && !S[i]) { //가장 기본식이 D[w] > D[i] 인데 여기서 D[w]나 D[i]가 0인 경우는 특수한 경우니 각각 추가 조건으로 해결. 아래 for문에서도 같은 방식.
                if (D[w] == 0 || D[w] > D[i]) w = i;
            }
        }

        S[w] = true;

        for (int i = 0; i < 9; ++i) {
            if (graph[w][i] != 0 && !S[i]) {
                if ((D[i] == 0) || (D[i] > D[w] + graph[w][i])) {
                    D[i] = D[w] + graph[w][i]; // w->i 엣지는 start->i로 가는 최단 경로 내에 포함된다.
                }
            }
        }
    }

    //출력
    cout << "Destination\tDistance from Source" << endl;

    for (int i = 0; i < 9; ++i) {
        cout << i << "\t\t" << D[i] << endl;
    }
}

int main()
{
    int graph[9][9] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
                        { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
                        { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
                        { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
                        { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
                        { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
                        { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
                        { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
                        { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };

    dijkstra(graph, 0);

    return 0;
}