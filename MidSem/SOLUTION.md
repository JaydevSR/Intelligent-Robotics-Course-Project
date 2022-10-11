# Solutions

## Problem 1: Time taken for coverage in trapezoidal decomposition
Given $\mathcal{S}$ be the total space including obstacles, we have the obstacles given by $\mathcal{O_i} \subset \mathcal{S}$ for $i = 1, 2, \dots, M$. Now, we define the set of traversable points as $\mathcal{T}$ which is the part of $\mathcal{S}$ that is disjoint from all the obstacles $O_i$. From this we generate a decomposition of $\mathcal T$ into trapazoids as follows:

$$\mathcal T = \bigcup_{i=1}^N T_i$$

$$T_i\cap T_j = \emptyset \text{ for any } i, j$$

Here each $T_i$ is a trapazoid. Now let $\tau(T_i, s_i, e_i)$ give the time taken to cover trapazoid $T_i$ when starting at point $s_i \in T_i$ and ending at point $e_i \in T_i$. Also let $\omega(s, e)$ give the time take to go from point $s$ to $e$. Then the total coverage time can be defined using a set of tuples $\{(T^{(1)}, s_1, e_1), \dots, (T^{(r)}, s_r, e_r) \}$ such that:
1. $r \ge N$
2. $\forall k \in \{1, \dots, r\}: T^{(k)} \in \{ T_1, \dots, T_N \}$
3. $\forall k \in \{1, \dots, r\}: s_k, e_k \in T^{(k)}$
4. $\forall i \in \{1, \dots, N\}: \exists k \in \{1, \dots, r\} \ \text{s.t.} \ T_i = T^{(k)}$

Using the above conditions we can be sure that each trapazoidal cell is covered atleat once. Now we can define the coverage time of $\mathcal T$ using this, say $C_\mathcal{T}$:

$$C_\mathcal{T} = \tau(T^{(1)}, s_1, e_1) + \omega(e_1, s_2) + \tau(T^{(2)}, s_2, e_2) + \dots + \omega(e_{r-1}, s_r) + \tau(T^{(r)}, s_2, e_2)$$

$$\implies C_\mathcal{T} = \sum_{i=1}^r \tau(T^{(i)}, s_i, e_i) + \sum_{i=1}^{r-1} \omega(e_i, s_{i+1})$$


## Problem 2: Optimization of coverage time

From the previous part we have the expression for coverage time $C_\mathcal{T}$ whose value we want to minimize. For this we note that the coverage time can be lower bounded by the time:

$$C_L = \sum_{i=1}^N \tau(T_i, s_i, e_i)$$

Here, $s_i$ and $e_i$ are some points in $T_i$. Now in order to optimize the coverage, we fix $r=N$ and search for points $s_1, \dots, s_N$ and $e_1, \dots, e_N$ such that $C_\mathcal{T}$ is minimum. This can be framed as a optimization problem as:

$$\frac{\partial C_\mathcal{T}}{\partial s_i} = 0 \hspace{1em} \text{where } s_i \in \mathcal{T}$$

$$\frac{\partial C_\mathcal{T}}{\partial e_i} = 0 \hspace{1em} \text{where } e_i \in \mathcal{T}$$

With constraint to ensure that every cell is covered once:

$$\exists k \in \{1, \dots, N\}:\ s_i \in T_k \iff e_i \in T_k$$

$$s_i \in T_k \implies s_j \notin T_k \hspace{1em} \text{if } i \ne j$$

## Problem 3: ...

## Probelm 4: Potential Field Path Planning

The path planner using potential fields is implemented in https://github.com/JaydevSR/Intelligent-Robotics-Course-Project/blob/3ae113f2ade5dfe4b99340675fe9c7235e4be471/MidSem/potential_field.py#L5-L46. Given below is the scenario with four circular obstacles. The parameters used for this were:

* Repulsive gain for obstacles = -1
* Attractive gain for goal = 2
* Step size for gradient descent = 0.05

![Path Planning Scenario](https://github.com/JaydevSR/Intelligent-Robotics-Course-Project/blob/main/MidSem/plots/potential_field_scenario.png?raw=true)

### Bonus

TODO

## Problem 5: Pursuit Evasion with two pursuers

Dropped

## Problem 6: Preventing pursuer collisions in multi-pursuer games

TODO

## Problem 7: A* Algorithm

The A* algorithm for finding the shortest path is implemented in https://github.com/JaydevSR/Intelligent-Robotics-Course-Project/blob/3ae113f2ade5dfe4b99340675fe9c7235e4be471/MidSem/shortest_path.py#L58-L87
