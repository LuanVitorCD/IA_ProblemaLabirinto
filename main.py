import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time
import random
import io

# ---------------------------------------------------------
# CONFIGURAÇÕES E ESTADOS DA SESSÃO
# ---------------------------------------------------------
st.set_page_config(page_title="Labirinto - Algoritmo A*", layout="wide", page_icon="⭐", initial_sidebar_state="expanded")

SIZE_OPTIONS = {"Pequeno": 10, "Médio": 20, "Grande": 35}
DEFAULT_SIZE_LABEL = "Médio"
DEFAULT_DENSITY_PCT = 25
BATCH_SIZE = 5       # nós processados entre cada atualização visual
FRAME_DELAY = 0.1    # atraso fixo (segundos) da animação

DIRECTIONS_ORTHO = ((-1, 0), (1, 0), (0, -1), (0, 1))
DIRECTIONS_DIAG = ((-1, -1), (-1, 1), (1, -1), (1, 1))
STEP_ORTHO = 1.0
STEP_DIAG = math.sqrt(2)

# Cores usadas na renderização (centralizadas para evitar "números mágicos" espalhados)
COLOR_WALL = (0.9, 0.9, 0.95)     # Obstáculo / Parede
COLOR_FREE = (0.08, 0.09, 0.11)   # Caminho livre
COLOR_CLOSED = (0.4, 0.1, 0.4)    # Nós explorados (fechados)
COLOR_OPEN = (0.1, 0.4, 0.1)      # Fronteira (abertos)
COLOR_PATH = (0.37, 0.63, 1.0)    # Caminho final encontrado
COLOR_START = (0.0, 1.0, 0.0)     # Início
COLOR_END = (1.0, 0.3, 0.3)       # Destino


def generate_solvable_maze(rows, cols, density=0.25):
    """
    Gera um labirinto GARANTIDAMENTE SOLUCIONÁVEL usando Busca em Profundidade (DFS)
    para criar uma árvore de caminhos, e depois quebra paredes para atingir a densidade.
    """
    grid = np.ones((rows, cols), dtype=int)

    stack = [(0, 0)]
    visited = {(0, 0)}
    grid[0, 0] = 0

    while stack:
        r, c = stack[-1]
        unvisited_neighbors = []

        for dr, dc in DIRECTIONS_ORTHO:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in visited:
                visited_count = 0
                for ddr, ddc in DIRECTIONS_ORTHO:
                    nnr, nnc = nr + ddr, nc + ddc
                    if 0 <= nnr < rows and 0 <= nnc < cols and grid[nnr, nnc] == 0:
                        visited_count += 1

                if visited_count <= 1:
                    unvisited_neighbors.append((nr, nc))

        if unvisited_neighbors:
            nr, nc = random.choice(unvisited_neighbors)
            visited.add((nr, nc))
            grid[nr, nc] = 0
            stack.append((nr, nc))
        else:
            stack.pop()

    grid[rows - 1, cols - 1] = 0
    if cols > 1:
        grid[rows - 1, cols - 2] = 0
    if rows > 1:
        grid[rows - 2, cols - 1] = 0

    target_walls = int(rows * cols * density)
    current_walls = np.sum(grid == 1)

    if current_walls > target_walls:
        wall_coords = np.argwhere(grid == 1)
        np.random.shuffle(wall_coords)

        walls_to_remove = current_walls - target_walls
        for i in range(walls_to_remove):
            r, c = wall_coords[i]
            grid[r, c] = 0

    grid[0, 0] = 0
    grid[rows - 1, cols - 1] = 0

    return grid


def _init_state():
    """Garante que todas as chaves de sessão existam antes de qualquer widget ser criado."""
    defaults = {
        "start_node": (0, 0),
        "end_node": (SIZE_OPTIONS[DEFAULT_SIZE_LABEL] - 1, SIZE_OPTIONS[DEFAULT_SIZE_LABEL] - 1),
        "cfg_size": DEFAULT_SIZE_LABEL,
        "cfg_density": DEFAULT_DENSITY_PCT,
        "diagonal_enabled": False,
        "reset_requested": False,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value

    if "grid" not in st.session_state:
        n = SIZE_OPTIONS[DEFAULT_SIZE_LABEL]
        st.session_state.grid = generate_solvable_maze(n, n, DEFAULT_DENSITY_PCT / 100.0)


def apply_maze_config(size_label, density_pct):
    """Regera o labirinto a partir da configuração pendente. Só deve ser chamada ao clicar em Aplicar/Resetar."""
    n = SIZE_OPTIONS[size_label]
    st.session_state.grid = generate_solvable_maze(n, n, density_pct / 100.0)
    st.session_state.start_node = (0, 0)
    st.session_state.end_node = (n - 1, n - 1)


def calc_heuristic(a, b, diagonal):
    """H(n): Manhattan (movimento ortogonal) ou Octile (movimento com diagonais)."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
    return dx + dy


def get_neighbors(node, rows, cols, grid, diagonal):
    """Retorna [(vizinho, custo_do_passo), ...] válidos a partir do nó atual."""
    r, c = node
    neighbors = []

    for dr, dc in DIRECTIONS_ORTHO:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
            neighbors.append(((nr, nc), STEP_ORTHO))

    if diagonal:
        for dr, dc in DIRECTIONS_DIAG:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                # Evita "cortar quina": só permite diagonal se as duas células ortogonais vizinhas forem livres
                if grid[r + dr, c] == 0 and grid[r, c + dc] == 0:
                    neighbors.append(((nr, nc), STEP_DIAG))

    return neighbors


def astar_search(grid, start, end, diagonal):
    """
    Gerador que executa o A* passo a passo, cedendo o estado da busca a cada nó
    expandido. Separar o algoritmo da renderização deixa cada parte mais fácil de
    entender e testar.
    """
    rows, cols = grid.shape
    counter = 0  # desempate estável no heap (evita comparar as tuplas dos nós)

    h_start = calc_heuristic(start, end, diagonal)
    open_list = [(h_start, counter, start)]
    came_from = {}
    g_score = {start: 0}
    costs = {start: {"g": 0, "h": h_start, "f": h_start}}
    open_set = {start}
    closed_set = set()
    explored = 0

    while open_list:
        _, _, current = heapq.heappop(open_list)
        if current in closed_set:
            continue
        open_set.discard(current)

        if current == end:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            yield {"open_set": open_set, "closed_set": closed_set, "costs": costs,
                   "explored": explored, "done": True, "found": True, "path": path}
            return

        closed_set.add(current)
        explored += 1

        for neighbor, step_cost in get_neighbors(current, rows, cols, grid, diagonal):
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = calc_heuristic(neighbor, end, diagonal)
                f = tentative_g + h
                costs[neighbor] = {"g": tentative_g, "h": h, "f": f}

                if neighbor not in open_set:
                    open_set.add(neighbor)
                    counter += 1
                    heapq.heappush(open_list, (f, counter, neighbor))

        yield {"open_set": open_set, "closed_set": closed_set, "costs": costs,
               "explored": explored, "done": False}

    yield {"open_set": open_set, "closed_set": closed_set, "costs": costs,
           "explored": explored, "done": True, "found": False, "path": []}


# ---------------------------------------------------------
# RENDERIZAÇÃO GRÁFICA
# ---------------------------------------------------------
def build_base_image(grid):
    """Constrói (de forma vetorizada) a imagem estática de paredes/caminhos do grid."""
    img = np.empty((*grid.shape, 3))
    img[grid == 1] = COLOR_WALL
    img[grid == 0] = COLOR_FREE
    return img


def create_maze_figure(rows, cols):
    """Cria a figura, os eixos e a imagem UMA única vez por execução da busca."""
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(8, 8), layout="tight")
    fig.patch.set_facecolor("#0e1117")
    ax.set_facecolor("#0e1117")

    im = ax.imshow(np.zeros((rows, cols, 3)))

    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="#333333", linestyle="-", linewidth=1)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    return fig, ax, im


def update_maze_figure(ax, im, base_img, start, end, path, open_set, closed_set, costs, show_costs, text_artists):
    """Atualiza a imagem e os textos de custo (F, G, H) sem recriar a figura inteira."""
    img = base_img.copy()

    if closed_set:
        rs, cs = zip(*closed_set)
        img[list(rs), list(cs)] = COLOR_CLOSED
    if open_set:
        rs, cs = zip(*open_set)
        img[list(rs), list(cs)] = COLOR_OPEN
    if path:
        rs, cs = zip(*path)
        img[list(rs), list(cs)] = COLOR_PATH

    img[start] = COLOR_START
    img[end] = COLOR_END
    im.set_data(img)

    for artist in text_artists:
        artist.remove()
    text_artists.clear()

    rows = base_img.shape[0]
    if show_costs and rows <= 25:
        relevant = open_set | closed_set | set(path)
        for (r, c) in relevant:
            vals = costs.get((r, c))
            if not vals:
                continue
            f_val = f"{vals['f']:.1f}" if isinstance(vals["f"], float) else str(vals["f"])
            g_val = f"{vals['g']:.1f}" if isinstance(vals["g"], float) else str(vals["g"])
            h_val = f"{vals['h']:.1f}" if isinstance(vals["h"], float) else str(vals["h"])

            text_artists.append(ax.text(c, r, f_val, ha="center", va="center",
                                         color="white", fontsize=8, fontweight="bold"))
            text_artists.append(ax.text(c - 0.4, r - 0.35, g_val, ha="left", va="top",
                                         color="#ffb266", fontsize=6))
            text_artists.append(ax.text(c + 0.4, r - 0.35, h_val, ha="right", va="top",
                                         color="#66ff66", fontsize=6))


def render_metrics(metric_path, metric_explored, metric_status, state):
    """Atualiza os três cartões de estatística a partir do estado mais recente da busca."""
    if state is None:
        metric_path.metric("Tamanho do Caminho", "-")
        metric_explored.metric("Nós Explorados", "0")
        metric_status.metric("Status", "Aguardando")
        return

    metric_explored.metric("Nós Explorados", state["explored"])
    if not state["done"]:
        metric_status.metric("Status", "Buscando...", delta_color="off")
    elif state["found"]:
        metric_path.metric("Tamanho do Caminho", len(state["path"]) - 1, delta="Concluído")
        metric_status.metric("Status", "Caminho Encontrado")
    else:
        metric_path.metric("Tamanho do Caminho", "Inalcançável")
        metric_status.metric("Status", "Falhou")


# ---------------------------------------------------------
# INTERFACE PRINCIPAL
# ---------------------------------------------------------
def main():
    _init_state()

    # Um reset pendente precisa ser aplicado ANTES de os widgets de configuração serem criados
    if st.session_state.reset_requested:
        st.session_state.cfg_size = DEFAULT_SIZE_LABEL
        st.session_state.cfg_density = DEFAULT_DENSITY_PCT
        apply_maze_config(st.session_state.cfg_size, st.session_state.cfg_density)
        st.session_state.reset_requested = False

    primary_color = st.get_option("theme.primaryColor")

    # A responsividade das imagens ainda precisa de CSS manual; cores/tema agora vêm do .streamlit/config.toml
    st.markdown(
        """
        <style>
        .block-container { padding-top: 2rem !important; padding-bottom: 0rem !important; }
        div { text-align: left; }
        [data-testid="stImage"] { display: flex; justify-content: center; align-items: center; width: 100% !important; }
        [data-testid="stImage"] img {
            max-height: 75vh !important;
            width: auto !important;
            object-fit: contain !important;
            border-radius: 8px;
        }
        section[data-testid="stSidebar"] {
            width: 450px !important;
        }
        section[data-testid="stSidebar"] > div:not([data-testid="stSidebarContent"]) {
            display: none !important;
            width: 0px !important;
            pointer-events: none !important;
        }
        div[data-testid="stSidebarCollapseButton"] {
            display: inline !important;
            visibility: inline !important;
        }
        </style>
        """,
        unsafe_allow_html=True,
    )

    title_html = f"""
                <div style='background-color: #1e1e1e; padding: 5px; border-radius: 5px; border-left: 4px solid {primary_color};'>
                    <h1 style='font-size: 32px; margin-left: 10px;'><b>Solucionador de Labirinto</b><br>
                        <i style='font-size: 20px; margin-left: 10px;'><b style='color: {primary_color};'>Grupo: </b> Ana, Luan e Wesley</i>
                    </h1>
                </div>
                """
    st.sidebar.markdown(title_html, unsafe_allow_html=True)
    st.sidebar.markdown("<br>", unsafe_allow_html=True)

    info_html = """
            <div style='background-color: #1e1e1e; padding: 15px; border-radius: 5px; border-left: 2px solid #5ea1ff; font-size: 16px; margin-bottom: 15px;'>
                <b style='color: #5ea1ff;'>Busca Informada: Algoritmo A*</b><br>
                O algoritmo A* possui complexidade de tempo de <b>O(b^d)</b> no pior caso, onde <i>b</i> é o fator de ramificação e <i>d</i> a profundidade da solução.<br><br>
                Para encontrar o menor caminho eficientemente, o A* avalia <b>F(n) = G(n) + H(n)</b> para cada quadro:<br>
                🔹 <b>G(n)</b>: Custo exato desde a origem.<br>
                🔹 <b>H(n)</b>: Estimativa heurística até o destino.
            </div>
            """
    with st.sidebar.expander("Explicação e Complexidade", expanded=False, icon="ℹ️"):
        st.markdown(info_html, unsafe_allow_html=True)

    st.sidebar.divider()

    # ---------------------------------------------------------
    # TÉCNICAS DE BUSCA E CONTROLE
    # ---------------------------------------------------------
    st.sidebar.subheader("Técnicas de Busca")
    diagonal_enabled = st.sidebar.checkbox(
        "↗️ Permitir movimento diagonal", value=False, key="diagonal_enabled",
        help="Desligado: heurística de Manhattan (4 direções). Ligado: heurística Octile (8 direções).",
    )
    algo_label = "Octile (8 direções)" if diagonal_enabled else "Manhattan (4 direções)"
    st.sidebar.caption(f"Heurística ativa: **{algo_label}**")
    
    animar = st.sidebar.checkbox("▶️ Ativar Animação", value=True, help="Visualiza o progresso do algoritmo passo a passo.")

    st.sidebar.divider()

    start_btn = st.sidebar.button("Iniciar Busca (A*)", type="primary", use_container_width=True)

    st.sidebar.divider()

    # ---------------------------------------------------------
    # CONFIGURAÇÕES AVANÇADAS
    # ---------------------------------------------------------
    with st.sidebar.popover("⚙️ Configurações Avançadas", use_container_width=True):
        with st.form("maze_config_form"):
            st.subheader("Mapa do Labirinto")
            st.radio("Tamanho", list(SIZE_OPTIONS.keys()), key="cfg_size", horizontal=True)
            st.slider("Densidade de Obstáculos (%)", min_value=0, max_value=50, step=5, key="cfg_density")

            col_apply, col_reset = st.columns(2)
            aplicar = col_apply.form_submit_button("✅ Aplicar", type="primary", use_container_width=True)
            resetar = col_reset.form_submit_button("↺ Resetar", use_container_width=True)

        st.divider()
        st.subheader("Variáveis Visuais")
        show_costs = st.toggle("Exibir Cálculos nos Quadros (F, G, H)", value=True, key="cfg_show_costs")
        if show_costs and st.session_state.grid.shape[0] > 25:
            st.warning("Visualizar os custos pode poluir a tela em grids maiores que 25x25.")
        st.info(
            f"Com a configuração atual, a busca usa a heurística **{algo_label}**. "
            "Ative o movimento diagonal ao lado para permitir atalhos na diagonal."
        )

    if aplicar:
        apply_maze_config(st.session_state.cfg_size, st.session_state.cfg_density)
    elif resetar:
        st.session_state.reset_requested = True
        st.rerun()

    # Preparação dos dados base para renderização
    grid = st.session_state.grid
    rows, cols = grid.shape
    start_node = st.session_state.start_node
    end_node = st.session_state.end_node
    base_img = build_base_image(grid)

    fig, ax, im = create_maze_figure(rows, cols)
    text_artists = []

    col_metrics, col_chart = st.columns([1, 3])

    with col_metrics:
        st.subheader("Estatísticas")
        metric_path = st.empty()
        metric_explored = st.empty()
        metric_status = st.empty()

    with col_chart:
        chart_placeholder = st.empty()

    # ---------------------------------------------------------
    # LAÇO INTERNO DO ALGORITMO (Sem recarregar a página)
    # ---------------------------------------------------------
    if start_btn:
        state = None
        for state in astar_search(grid, start_node, end_node, diagonal_enabled):
            # Renderização animada quadro a quadro
            if animar and not state["done"]:
                if state["explored"] % BATCH_SIZE == 0:
                    render_metrics(metric_path, metric_explored, metric_status, state)
                    update_maze_figure(ax, im, base_img, start_node, end_node, [], state["open_set"], state["closed_set"], state["costs"], show_costs, text_artists)
                    
                    # Salva a imagem em memória para manter o CSS ativo sem piscar a tela
                    buf = io.BytesIO()
                    fig.savefig(buf, format="png", bbox_inches="tight", facecolor=fig.get_facecolor(), pad_inches=0.1)
                    buf.seek(0)
                    chart_placeholder.image(buf)
                    
                    time.sleep(FRAME_DELAY)

        # Atualizações finais (ocorre independentemente da animação estar ligada ou desligada)
        render_metrics(metric_path, metric_explored, metric_status, state)
        
        if state["found"]:
            st.success(f"Busca finalizada! O caminho mais curto foi encontrado com custo de {len(state['path']) - 1} passos.")
        else:
            st.error("Não há caminho possível para o destino. O labirinto está fechado.")

        path = state["path"] if (state and state["done"]) else []
        open_set = state["open_set"] if state else set()
        closed_set = state["closed_set"] if state else set()
        costs = state["costs"] if state else {}
        
        update_maze_figure(ax, im, base_img, start_node, end_node, path, open_set, closed_set, costs, show_costs, text_artists)
        
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", facecolor=fig.get_facecolor(), pad_inches=0.1)
        buf.seek(0)
        chart_placeholder.image(buf)
        
    else:
        # Mostra o labirinto ocioso enquanto o usuário não clica em Iniciar
        update_maze_figure(ax, im, base_img, start_node, end_node, [], set(), set(), {}, show_costs, text_artists)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", bbox_inches="tight", facecolor=fig.get_facecolor(), pad_inches=0.1)
        buf.seek(0)
        chart_placeholder.image(buf)
        render_metrics(metric_path, metric_explored, metric_status, None)

    plt.close(fig)


if __name__ == "__main__":
    main()