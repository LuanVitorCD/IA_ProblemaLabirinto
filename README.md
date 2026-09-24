# Problema do Labirinto com Algoritmo A*

## 📌 Sobre o Projeto

O problema de *Pathfinding* (busca de caminhos) consiste em navegar por um ambiente com obstáculos para determinar a rota mais eficiente entre dois pontos. Neste projeto, o objetivo é encontrar o caminho mais curto em um labirinto (grid), partindo de um ponto de origem até um destino, utilizando o algoritmo de busca informada **A*** **(A Estrela)**. No seu pior cenário possui a complexidade exponencial $O(b^d)$.

Esse algoritmo é amplamente reconhecido na área de Inteligência Artificial e no desenvolvimento de jogos pela sua eficiência e precisão na determinação do menor trajeto. O projeto demonstra visualmente como o algoritmo toma decisões, avaliando caminhos promissores através de cálculos de custo e estimativas matemáticas (heurísticas), descartando rotas ineficientes.

## ⭐ Definição do Algoritmo A*

O A* (A Estrela) é um algoritmo de busca em grafos que encontra o caminho de menor custo de um nó inicial até um nó objetivo. Ele se destaca por ser um exemplo clássico de **busca informada** (ou heurística). 

Diferente de algoritmos cegos (como a busca em largura) que exploram todas as direções igualmente, o A* é "inteligente" e direcionado. Ele decide qual nó explorar a seguir baseando-se em uma função matemática $F(n)$ associada a cada nó (célula) do grid:

$$F(n) = G(n) + H(n)$$

Onde:
*   **$G(n)$ (Custo Exato / Custo do Caminho):** É o custo real (distância percorrida) desde o ponto de início (origem) até o nó atual $n$. Ele é acumulativo. Se moverse ortogonalmente custa $1$ e diagonalmente custa $\approx 1.41$ ($\sqrt{2}$), o custo de chegar em uma célula é o custo para chegar no vizinho anterior somado ao custo de dar esse novo passo.
*   **$H(n)$ (Estimativa / Heurística):** É o "palpite" inteligente. É a estimativa do custo para ir do nó atual $n$ até o destino final, ignorando possíveis obstáculos no meio.
*   **$F(n)$ (Custo Total Estimado):** É a soma de $G(n)$ e $H(n)$. O algoritmo sempre escolhe o próximo nó para processar olhando para o **menor valor de $F(n)$** disponível.

### A Estrutura de Listas (Abertos e Fechados)

Para funcionar, o A* gerencia duas coleções de nós:
1.  **Lista Aberta (Fronteira):** Os nós que foram descobertos, tiveram seus custos ($F, G, H$) calculados, mas ainda não foram visitados (expandidos). O algoritmo sempre busca aqui o nó com o menor $F$ para ser o próximo nó atual.
2.  **Lista Fechada (Explorados):** Os nós que o algoritmo já visitou, expandiu (olhou para seus vizinhos) e decidiu que já encontrou a melhor forma possível de chegar até eles.

### O Ciclo de Busca (Como Funciona)

1.  Inicia-se adicionando o nó de origem na **Lista Aberta**.
2.  **Laço Principal:** Enquanto houver nós na **Lista Aberta**:
    *   Retira-se o nó com o menor custo $F(n)$ da lista aberta e o define como **Nó Atual**.
    *   Adiciona-se esse **Nó Atual** à **Lista Fechada**.
    *   Se o **Nó Atual** for o destino, a busca terminou com sucesso (caminho encontrado).
    *   Para cada nó vizinho válido (que não seja parede e não esteja na lista fechada):
        *   Calcula-se o novo $G(n)$ (o $G$ do pai + custo do passo).
        *   Se o vizinho ainda não está na **Lista Aberta**, ele é adicionado. O nó atual é salvo como sendo seu "Pai" (necessário para traçar a rota final). Calcula-se $H(n)$ e $F(n)$.
        *   Se o vizinho já estava na **Lista Aberta**, verifica-se se o novo $G(n)$ calculado pelo caminho atual é *menor* que o $G(n)$ antigo. Se sim, significa que encontramos um "atalho" para esse vizinho. Atualiza-se o "Pai" para o nó atual e recalcula-se $F(n)$.
3.  Se a **Lista Aberta** esvaziar e o destino não tiver sido alcançado, o caminho está bloqueado (sem solução).

## ⚙️ Implementação do Algoritmo A*

O núcleo matemático do algoritmo encontra-se em `main.py`, desacoplado da lógica de exibição, utilizando uma função geradora (yield) para permitir a animação passo a passo da interface.

Abaixo, detalhamos a implementação:

### 1. Custos de Movimentação (Constantes)
*   **Código:** `STEP_ORTHO = 1.0` e `STEP_DIAG = math.sqrt(2)`
*   **Explicação:** O custo de mover-se para as laterais/cima/baixo é exato ($1$). Se a opção de diagonal estiver ativa na interface, o passo diagonal custa a raiz de 2 ($\approx 1.414$), pois a diagonal de um quadrado de lado $1$ pelo Teorema de Pitágoras $h = \sqrt{a^2 + b^2} \rightarrow h = \sqrt{1^2 + 1^2} = \sqrt{2}$.

### 2. A Heurística ($H$)
*   **Código:** Função `calc_heuristic(a, b, diagonal)`
*   **Explicação:** Calcula a estimativa. 
    *   Se o usuário opta por **Movimento Ortogonal (4 direções)**, utiliza-se a heurística de **Manhattan**, calculando a distância absoluta apenas em L. 
    *   Se o usuário opta por **Movimento Diagonal (8 direções)**, utiliza-se a heurística de **Octile**, que permite movimentos diagonais no cálculo da estimativa teórica, melhorando drasticamente a precisão da busca quando as diagonais são liberadas no ambiente.

### 3. As Estruturas de Dados
*   **Código:** Inicializadas dentro de `astar_search()`
    *   `open_list` (HeapQ): Usamos a biblioteca `heapq` do Python para estruturar a lista aberta como uma Fila de Prioridade (Min-Heap). Isso garante que extrair o nó com o **menor valor de $F(n)$** custe apenas $O(\log n)$, em vez de $O(n)$ de uma busca linear em lista, otimizando o algoritmo massivamente.
    *   `closed_set` (Set): Utilizamos um `set` (Conjunto Hash) para a lista fechada, garantindo que a verificação "O nó está na lista fechada?" demore $O(1)$ ao invés de $O(n)$.
    *   `came_from` (Dicionário): Guarda de qual nó pai o nó atual veio (para rastrear o caminho final).
    *   `g_score` (Dicionário): Guarda o melhor valor $G$ conhecido para cada nó.

### 4. Avaliação de Vizinhos
*   **Código:** Função `get_neighbors(node, rows, cols, grid, diagonal)`
*   **Explicação:** Verifica as direções possíveis. Se bater numa parede (`grid == 1`) ou sair dos limites, a opção é descartada. No caso de diagonais ativas, há uma trava (anti-corte-de-quinas) para que o algoritmo não passe espremido "por dentro" de duas paredes adjacentes.

### 5. O Fluxo de Execução
*   **Código:** Função principal `astar_search(grid, start, end, diagonal)`
    *   Enquanto o `open_list` (min-heap) tem nós, o nó de menor $F$ é removido via `heapq.heappop()`.
    *   Ele é salvo no `closed_set`.
    *   Itera-se pelos vizinhos gerados pela função acima.
    *   Calcula-se o caminho tentativo (`tentative_g = g_score[current] + step_cost`).
    *   Se for um caminho melhor (`tentative_g < g_score`), ou se o nó é inédito, atualiza os dicionários com o novo "pai", novo $G$, a nova heurística calculada $H$, e o $F$ somado. Adiciona no `open_set` (e no heap).
    *   Quando o `current == end`, o percurso é traçado de trás para frente usando o dicionário `came_from`.

## 🛠️ Tecnologias Utilizadas

*   **Python:** Linguagem responsável pela lógica e estrutura de dados.
*   **Streamlit:** Framework para a construção da interface gráfica interativa web, menus e botões.
*   **NumPy:** Utilizado para a manipulação de matrizes, permitindo que a malha do labirinto (grid) seja processada de forma rápida.
*   **Matplotlib:** Responsável pela plotagem dos gráficos (mapa do labirinto) e sobreposição do caminho, de forma que seja dinamicamente atualizado no Streamlit via arrays do NumPy e `BytesIO`.
*   **HeapQ:** Biblioteca padrão Python para implementação eficiente da Fila de Prioridade na Lista Aberta.

## 🚀 Funcionalidades da Aplicação

*   **Geração Dinâmica do Labirinto:** Cria labirintos que são garantidamente solucionáveis e permite que o usuário adicione ou diminua a densidade (quantidade) de obstáculos de forma visual, controlando também o tamanho do grid.
*   **Animação da Busca:** Acompanhe passo a passo, em tempo real, quais nós estão sendo avaliados (amarelo), explorados (roxo) e o caminho sendo trilhado.
*   **Exibição dos Cálculos:** Nos mapas menores, é possível ativar a visão avançada que escreve por cima do grid, em tempo real, os valores de F, G e H ($F(n) = G(n) + H(n)$) daquele exato ponto avaliado, como um modo "debug" visual do algoritmo.
*   **Permissão de Movimento Diagonal:** Com o apertar de um botão, o usuário permite que o algoritmo use movimentos em diagonal para cortar caminho (e a IA automaticamente altera o cálculo da heurística correspondente).

## 📂 Organização do Projeto

A estrutura de arquivos do projeto é:

```text
IA_ProblemaLabirinto/
├── .streamlit/          
│   └── config.toml      # Configurações de tema e exibição do Streamlit
├── .gitignore           # Arquivos ignorados pelo controle de versão do Git
├── main.py              # Código principal com a lógica do A* e interface
├── README.md            # Este arquivo de documentação
└── requirements.txt     # Dependências necessárias para rodar o projeto
```

## 💻 Execução Local

Siga os passos abaixo para executar a aplicação no seu computador:

1.  **Clone este repositório:**
    ```bash
    git clone https://github.com/LuanVitorCD/IA_ProblemaLabirinto.git
    cd IA_ProblemaLabirinto
    ```

2.  **Crie e ative um ambiente virtual (recomendado):**
    *   No Windows:
        ```bash
        python -m venv venv
        venv\Scripts\activate
        ```
    *   No Linux/Mac:
        ```bash
        python3 -m venv venv
        source venv/bin/activate
        ```

3.  **Instale as dependências:**

    Certifique-se de estar com o ambiente virtual ativado e execute:
    ```bash
    pip install -r requirements.txt
    ```
    *(Nota: se o arquivo não existir, instale manualmente: `pip install streamlit numpy matplotlib`)*

4.  **Execute a aplicação:**
    ```bash
    streamlit run main.py
    ```

5.  **Acesse a interface:**
    O Streamlit abrirá uma aba no seu navegador (geralmente `http://localhost:8501`).
