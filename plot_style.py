"""Visual style and color palette for all ML_V2 figures (7-model benchmark)."""
import matplotlib as mpl

# Distinct colors for the 7 models
C_LOGREG = "#7c3aed"    # Purple
C_KNN = "#d97706"       # Amber / Gold
C_DT = "#059669"        # Emerald
C_GNB = "#64748b"       # Slate
C_MLP = "#db2777"       # Magenta / Deep Pink
C_RF = "#2563eb"        # Royal Blue
C_XGB = "#ea580c"       # Orange-Red

# 4-model legacy colors
C_RF_CLEAN = "#1e40af"
C_RF_NOISY = "#0d9488"
C_XGB_CLEAN = "#ea580c"
C_XGB_NOISY = "#e11d48"

MODEL_COLORS = {
    "rf_clean": C_RF_CLEAN,
    "rf_noisy": C_RF_NOISY,
    "xgb_clean": C_XGB_CLEAN,
    "xgb_noisy": C_XGB_NOISY,
}

MODEL_LABELS = {
    "rf_clean": "Random Forest (Clean)",
    "rf_noisy": "Random Forest (Noisy)",
    "xgb_clean": "XGBoost (Clean)",
    "xgb_noisy": "XGBoost (Noisy)",
}
# General purpose colors
C_MAIN = "#2563eb"
C_ALT = "#16a34a"
C_MUTED = "#94a3b8"
C_FAULT = "#dc2626"
C_GRID = "#e2e8f0"

MODELS_7 = [
    "logistic_regression",
    "naive_bayes",
    "knn",
    "decision_tree",
    "mlp",
    "random_forest",
    "xgboost",
]

MODEL_COLORS_7 = {
    "logistic_regression": C_LOGREG,
    "naive_bayes": C_GNB,
    "knn": C_KNN,
    "decision_tree": C_DT,
    "mlp": C_MLP,
    "random_forest": C_RF,
    "xgboost": C_XGB,
}

MODEL_LABELS_7 = {
    "logistic_regression": "Logistic Regression (Linear)",
    "naive_bayes": "Gaussian Naive Bayes (Probabilistic)",
    "knn": "K-Nearest Neighbors (Instance-based)",
    "decision_tree": "Decision Tree (Single CART)",
    "mlp": "Multi-Layer Perceptron (Neural Net)",
    "random_forest": "Random Forest (300 Trees)",
    "xgboost": "XGBoost (300 Rounds)",
}

MODEL_SHORT_7 = {
    "logistic_regression": "LogReg",
    "naive_bayes": "GaussianNB",
    "knn": "KNN",
    "decision_tree": "DecisionTree",
    "mlp": "MLP (NeuralNet)",
    "random_forest": "RandomForest",
    "xgboost": "XGBoost",
}


def apply_style():
    mpl.rcParams.update({
        "figure.dpi": 110,
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.color": C_GRID,
        "grid.linewidth": 0.8,
        "axes.axisbelow": True,
        "legend.frameon": False,
    })
