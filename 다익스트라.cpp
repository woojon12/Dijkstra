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
    queue<int> P[9]; /* path : 0~8 까지의 정점으로 가는 최단 경로들
                     (큐라는 자료구조가 알아서 하겠지만 아무리 많은 정점을 지나도 한 경로 당 모든 정점 개수 - 시작정점 하나 까지가 최대임.)
                     일단 최단 거리가 두 개 이상 있는 경우는 신경쓰지 않고 코드 짬. (가장 이상적인 건 공동 1등 경로를 모두 출력하는건데 귀찮음)
                     사용법 : 첫 자리에 -1이 저장되있으면 경로가 없는 거임.
                     간선 하나가 발견될 때마다 그 도착점의 넘버를 계속 push한다.
                     계속 끝자리에 쌓으면서 넣어야 하다보니 queue가 필요.
                     더 짧은 경로가 발견되면 아예 처음부터 끝까지 재push.
                     */

    //초기 과정
    S[start] = true; //출력할 때 자기 자신과의 거리는 0이라 출력해야하는데 아래 for문(j)에서는 0이 제일 크다는 개념왜곡 때문에 자기 자신은 저 for문에서 조사받지 말아야 됨.
    
    for (int i = 0; i < 9; ++i) {
        if(graph[start][i] != 0) P[i].push(i); //start 정점이랑 i 정점이 연결되어있으면(!= 0) 다음 목표정점은 i 정점으로 저장, 아니면 empty 상태로 놔둠.
    }

    for (int i = 0; i < 9; ++i) {
        D[i] = graph[start][i]; //일단 냅다 시작점 기준으로 최단거리 잡기. [n][m]은 n으로부터 m까지의 거리임.
    }


    //본론 과정
    for (int j = 1; j < 9; ++j) { //정확히는 S의 모든 원소가 true가 될때까지만 반복인데 어차피 한번 할때마다 무조건 하나씩 true로 바뀌어서 (1+)8번이면 됨.
        int w = start; //복잡하게 코드짜면 안그래도 되지만 코드 간소화 하느라 0이 제일 크다는 개념왜곡의 영향으로 D[w]가 0인 w로 계속 초기화 해줘야 하는데 그런 자리는 start 임.

        for (int i = 0; i < 9; ++i) { //w를 정하기
            if (D[i] != 0 && !S[i]) { //가장 기본식이 D[w] > D[i] 인데 여기서 D[w]나 D[i]가 0인 경우는 특수한 경우니 각각 추가 조건으로 해결. 아래 for문에서도 같은 방식.
                if (D[w] == 0 || D[w] > D[i]) w = i; //근데 이거 그냥 D[i]가 0이 아닐때란 조건 같이 넣으면 되는 거 아님?
            }
        }

        S[w] = true;

        for (int i = 0; i < 9; ++i) {
            if (graph[w][i] != 0 && !S[i]) {
                if ((D[i] == 0) || (D[i] > D[w] + graph[w][i])) {
                    if (D[w] != 0) { //w 정점까지 오는 길이 진정 있는 게 맞으면

                        //아래 if문은 기존 경로가 있으면 비교 작업을 하고 없으면(else) 걍 push 하는 거임.
                        if (!P[i].empty()) {
                            queue<int> pre_path = P[i]; //지금까지 밝혀진 i 정점까지 오는 최단경로를 분석하기 위해 임시저장
                            int prev, next;
                            int pre_d = 0, post_d = 0;

                            prev = start;

                            while (!pre_path.empty()) {
                                next = pre_path.front();
                                pre_d += graph[prev][next];
                                prev = next;
                                pre_path.pop();
                            }

                            queue<int> post_path = P[w]; //이제 막 밝혀진 i 정점까지 오는 경로가 기존 경로보다 더 짧은지 분석하기 위해 임시저장. 여기다 w->i 간선 가중치 더하면 완성임

                            prev = start;

                            while (!post_path.empty()) { //post_path(=P[w])는 w 정점까지 온 시점에서 반드시 존재하기에 사실 첫 pop에선 empty 검사 불필요. 흠 그럼 do while문이 조금 더 빠른가. 위도 그렇고.
                                next = post_path.front();
                                post_d += graph[prev][next];
                                prev = next;
                                post_path.pop();
                            }

                            post_d += graph[w][i];

                            if (pre_d > post_d) { //만약 새로 발견된 경로가 더 효율적인 짧은 거리면
                                //P[i]를 "P[w] + (w,i)간선"으로 완전 교체
                                // 큐 덮어씌우기에 관한 자료 : https://robodream.tistory.com/333 그냥 =를 쓰면 됨.
                                post_path = P[w];
                                post_path.push(i); //post_path에 pop작업을 했으니 다 날라가버려서 다시 복구하고 대입
                                P[i] = post_path;
                            }
                        }
                        else {
                            P[i] = P[w];
                            P[i].push(i);  // w->i 간선은 start->i로 가는 최단 경로 내에 포함된다.
                        }

                        D[i] = D[w] + graph[w][i];
                    }
                }
            }
        }
    }

    //출력
    cout << "Destination\tDistance from Source\tPath" << endl;

    for (int i = 0; i < 9; ++i) {
        cout << i << "\t\t" << D[i] << "\t\t\t";
        
        if (!P[i].empty()) {
            queue<int> temp = P[i];
            cout << start;

            do {
                cout << " - " << temp.front();
                temp.pop();
            } while (!temp.empty());

            cout << endl;
        }
        else {
            if (i != start) cout << "not existing Path" << endl;
            else cout << start << endl;
        }
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