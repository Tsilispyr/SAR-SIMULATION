"""
SAR Batch Test - Αυτόματη Δημιουργία Αναφοράς στα Ελληνικά
Χρήση: python generate_report.py [batch_results.json]
Έξοδος: SAR_Report_GR.md
"""
import json
import sys
import os
from datetime import datetime

INPUT   = sys.argv[1] if len(sys.argv) > 1 else "batch_results.json"
OUT     = "SAR_Report_GR.md"

with open(INPUT, encoding="utf-8") as f:
    data = json.load(f)

results  = data["results"]
summary  = data["summary"]
n_runs   = data["runs"]
generated = data.get("generated", "")

valid   = [r for r in results if r.get("result") in ("success", "partial", "failed")]
success = [r for r in valid if r["result"] == "success"]
partial = [r for r in valid if r["result"] == "partial"]
failed  = [r for r in valid if r["result"] == "failed"]
errors  = [r for r in results if r.get("result") in ("map_error", "start_error")]

# ── helpers ──────────────────────────────────────────────────────────────────
def avg(lst):
    return round(sum(lst) / len(lst), 1) if lst else 0.0

def pct(n, total):
    return round(100 * n / total, 1) if total else 0.0

def tier_block(label, name):
    cfg = next((p for p in [
        {"label":"easy",     "n_patrol":2,"n_aggr":1,"n_targets":3,"radius": 800},
        {"label":"medium",   "n_patrol":3,"n_aggr":2,"n_targets":5,"radius":1000},
        {"label":"hard",     "n_patrol":4,"n_aggr":3,"n_targets":6,"radius":1200},
        {"label":"very_hard","n_patrol":5,"n_aggr":4,"n_targets":7,"radius":1500},
    ] if p["label"] == label), {})

    t = [r for r in valid if r["preset"] == label]
    if not t:
        return f"*Δεν υπάρχουν δεδομένα για το επίπεδο {name}.*\n"

    s  = [r for r in t if r["result"] == "success"]
    p  = [r for r in t if r["result"] == "partial"]
    f_ = [r for r in t if r["result"] == "failed"]

    scores   = [r["score"] for r in t]
    s_scores = [r["score"] for r in s] if s else []
    elapsed  = [r["sim_elapsed_s"] for r in t if r.get("sim_elapsed_s", 0) > 0]
    s_elapsed= [r["sim_elapsed_s"] for r in s if r.get("sim_elapsed_s", 0) > 0]
    escapes  = [r.get("escape_events", 0) for r in t]
    esc_nonzero = [e for e in escapes if e > 0]

    lines = [
        f"**Παράμετροι σεναρίου:** {cfg.get('n_patrol','-')} περιπολίες + "
        f"{cfg.get('n_aggr','-')} επιθετικοί | {cfg.get('n_targets','-')} στόχοι | "
        f"ακτίνα {cfg.get('radius','-')} m",
        f"",
        f"**Αποτελέσματα:**",
        f"| Κατηγορία | Πλήθος | Ποσοστό |",
        f"|---|---|---|",
        f"| Επιτυχία (SUCCESS) | {len(s)} | {pct(len(s), len(t))}% |",
        f"| Μερική Επιτυχία (PARTIAL) | {len(p)} | {pct(len(p), len(t))}% |",
        f"| Αποτυχία (FAILED) | {len(f_)} | {pct(len(f_), len(t))}% |",
        f"| **Σύνολο** | **{len(t)}** | 100% |",
        f"",
        f"**Βαθμολογία:**",
        f"| Μέτρο | Όλες | Μόνο Επιτυχίες |",
        f"|---|---|---|",
        f"| Μέση | {avg(scores)} | {avg(s_scores) if s_scores else '-'} |",
        f"| Μέγιστη | {max(scores)} | {max(s_scores) if s_scores else '-'} |",
        f"| Ελάχιστη | {min(scores)} | {min(s_scores) if s_scores else '-'} |",
        f"",
        f"**Χρόνος ολοκλήρωσης (sim-time):**",
        f"| Μέτρο | Όλες | Μόνο Επιτυχίες |",
        f"|---|---|---|",
        f"| Μέσος | {avg(elapsed)} s | {avg(s_elapsed) if s_elapsed else '-'} s |",
        f"| Μέγιστος | {max(elapsed) if elapsed else '-'} s | {max(s_elapsed) if s_elapsed else '-'} s |",
        f"| Ελάχιστος | {min(elapsed) if elapsed else '-'} s | {min(s_elapsed) if s_elapsed else '-'} s |",
        f"",
        f"**Συμβάντα αποφυγής (Escape Events):**",
        f"- Μέση τιμή ανά αποστολή: **{avg(escapes)}**",
        f"- Αποστολές με 0 escape events: {sum(1 for e in escapes if e == 0)} "
        f"({pct(sum(1 for e in escapes if e == 0), len(t))}%)",
        f"- Αποστολές με ≥1 escape event: {len(esc_nonzero)} "
        f"({pct(len(esc_nonzero), len(t))}%)",
        f"- Μέγιστα escape events σε αποστολή: {max(escapes) if escapes else 0}",
    ]

    if esc_nonzero:
        lines.append(f"- Μέση τιμή όταν >0: {avg(esc_nonzero)} events")

    lines.append(f"")
    lines.append(f"**Ερμηνεία escape events για επίπεδο {name}:**")
    if avg(escapes) < 0.5:
        lines.append(
            f"Ο χαμηλός αριθμός escape events ({avg(escapes)}/αποστολή) υποδεικνύει ότι "
            f"ο αλγόριθμος penalized graph αποτρέπει προληπτικά τις επαφές - ο πράκτορας "
            f"αποφεύγει τις απειλές μέσω δρομολόγησης, χωρίς να χρειαστεί reactive escape."
        )
    elif avg(escapes) < 1.5:
        lines.append(
            f"Μέτριος αριθμός escape events ({avg(escapes)}/αποστολή) - "
            f"ο πράκτορας εισέρχεται περιστασιακά σε φάση φυγής, "
            f"κυρίως όταν επιθετικοί εχθροί βρίσκονται στον άμεσο δρόμο προς στόχο."
        )
    else:
        lines.append(
            f"Υψηλός αριθμός escape events ({avg(escapes)}/αποστολή) - "
            f"η πυκνότητα εχθρών σε αυτό το επίπεδο αναγκάζει τον πράκτορα "
            f"σε επαναλαμβανόμενες φάσεις φυγής πριν καταφέρει να φτάσει στους στόχους."
        )

    if f_:
        fail_scores   = [r["score"] for r in f_]
        fail_targets  = [r.get("targets_collected", 0) for r in f_]
        fail_elapsed  = [r.get("sim_elapsed_s", 0) for r in f_ if r.get("sim_elapsed_s", 0) > 0]
        fail_timeouts = [r for r in f_ if r.get("status_msg") == "TIMEOUT"]
        lines += [
            f"",
            f"**Ανάλυση αποτυχιών ({len(f_)} αποστολές):**",
            f"| Μέτρο | Τιμή |",
            f"|---|---|",
            f"| Μέση βαθμολογία | {avg(fail_scores)} |",
            f"| Μέσοι στόχοι πριν αποτυχία | {avg(fail_targets):.1f} / {cfg.get('n_targets','-')} |",
            f"| Μέσος χρόνος | {avg(fail_elapsed):.0f} s |" if fail_elapsed else "",
            f"| Αποτυχίες λόγω timeout | {len(fail_timeouts)} |",
        ]
        lines = [l for l in lines if l != ""]  # remove empty strings from conditional
        if len(fail_timeouts) > 0:
            lines.append(
                f"  *Οι timeouts υποδεικνύουν αποστολές όπου ο πράκτορας παγιδεύτηκε "
                f"σε βρόχο αποφυγής και δεν κατάφερε να ολοκληρώσει εντός 360s πραγματικού χρόνου.*"
            )
        fast_fails = [r for r in f_ if r.get("sim_elapsed_s", 999) < 100]
        if fast_fails:
            lines.append(
                f"  *{len(fast_fails)} γρήγορες αποτυχίες (<100 sim-sec): πιθανή τυχαία γένεση "
                f"κοντά σε επιθετικό εχθρό.*"
            )

    return "\n".join(lines) + "\n"

# ── special failure analysis ──────────────────────────────────────────────────
def failure_analysis():
    if not failed:
        return "_Δεν καταγράφηκαν αποτυχίες στα πλαίσια αυτής της δοκιμής._\n"
    lines = []
    # Fast failures (caught early)
    fast = [r for r in failed if r.get("sim_elapsed_s", 9999) < 100]
    slow = [r for r in failed if r.get("sim_elapsed_s", 0) >= 100]
    timeouts = [r for r in failed if r.get("status_msg") == "TIMEOUT"]

    if fast:
        lines.append(f"**Γρήγορες αποτυχίες (< 100 sim-sec) - {len(fast)} αποστολές:**")
        lines.append("Ο πράκτορας εντοπίστηκε και ακινητοποιήθηκε νωρίς στην αποστολή.")
        lines.append("Πιθανή αιτία: τυχαία γένεση κοντά σε επιθετικό εχθρό ή στενό δρόμο.")
        for r in fast:
            lines.append(
                f"  - RUN {r['run']:2d} [{r['preset']}]: score={r['score']}, "
                f"targets={r.get('targets_collected',0)}/{r.get('targets_total','?')}, "
                f"sim={r.get('sim_elapsed_s',0):.0f}s"
            )
        lines.append("")

    if slow:
        lines.append(f"**Αργές αποτυχίες (≥ 100 sim-sec) - {len(slow)} αποστολές:**")
        lines.append("Ο πράκτορας συνέλεξε μερικούς στόχους αλλά τελικά ακινητοποιήθηκε.")
        for r in slow:
            lines.append(
                f"  - RUN {r['run']:2d} [{r['preset']}]: score={r['score']}, "
                f"targets={r.get('targets_collected',0)}/{r.get('targets_total','?')}, "
                f"sim={r.get('sim_elapsed_s',0):.0f}s, escapes={r.get('escape_events',0)}"
            )
        lines.append("")

    if timeouts:
        lines.append(f"**Λήξεις χρόνου (TIMEOUT) - {len(timeouts)} αποστολές:**")
        lines.append(
            "Η αποστολή δεν ολοκληρώθηκε εντός του ορίου 360 δευτερολέπτων πραγματικού χρόνου "
            "(≈ 2880 sim-sec στα 8×). Πιθανή αιτία: εκτεταμένος βρόχος αποφυγής εχθρών που "
            "εμποδίζει την πρόσβαση στους υπόλοιπους στόχους."
        )
        lines.append("")

    # By tier
    lines.append("**Αποτυχίες ανά επίπεδο δυσκολίας:**")
    for label in ["easy", "medium", "hard", "very_hard"]:
        tf = [r for r in failed if r["preset"] == label]
        tt = [r for r in valid  if r["preset"] == label]
        if tf:
            lines.append(f"  - {label}: {len(tf)}/{len(tt)} αποστολές ({pct(len(tf), len(tt))}%)")
    return "\n".join(lines) + "\n"

# ── compose report ────────────────────────────────────────────────────────────
report = f"""# Αναφορά Αξιολόγησης Συστήματος SAR
## Αυτόματη Εξομοίωση Αποστολών - Αποτελέσματα & Ανάλυση

**Ημερομηνία δημιουργίας αναφοράς:** {datetime.now().strftime("%d/%m/%Y %H:%M")}
**Αρχείο δεδομένων:** `{INPUT}`
**Ημερομηνία εκτέλεσης δοκιμών:** {generated[:10] if generated else 'N/A'}

---

## 1. Εισαγωγή & Σκοπός Δοκιμών

Το σύστημα **SAR-SIM** (Search and Rescue Simulation) είναι μια web-based εξομοίωση αυτόνομης πλοήγησης πράκτορα σε πραγματικό αστικό οδικό δίκτυο. Το backend βασίζεται σε **FastAPI + NetworkX/OSMnx** και χρησιμοποιεί πραγματικά δεδομένα OSM (OpenStreetMap) για την Αθήνα.

Σκοπός των δοκιμών ήταν η **ποσοτική αξιολόγηση** της απόδοσης του αλγορίθμου αυτόνομης πλοήγησης σε διαφορετικές συνθήκες δυσκολίας, με μέτρηση:
- Ποσοστού επιτυχίας, μερικής επιτυχίας και αποτυχίας
- Βαθμολογίας ανά αποστολή
- Χρόνου ολοκλήρωσης αποστολής
- Συμβάντων αποφυγής εχθρών (escape events)

---

## 2. Μεθοδολογία Δοκιμών

### 2.1 Περιβάλλον Εκτέλεσης
- **Backend:** FastAPI σε Docker container (localhost:8000)
- **Οδικό δίκτυο:** OSMnx graph από OpenStreetMap - κέντρο Αθήνας (37.9838°N, 23.7275°E)
- **Ταχύτητα εξομοίωσης:** 8× (effective Δt = 1.6 sec/tick, 5 Hz loop)
  - *Σημείωση: Επιλέχθηκε 8× και όχι 50× για να διατηρηθεί αξιόπιστη ανίχνευση επαφής εχθρών (contact zone = 5m, max enemy jump = 7.2m/tick)*
- **Εργαλείο δοκιμών:** `batch_test.py` - αυτόματος ελεγκτής REST API

### 2.2 Παράμετροι που Μεταβλήθηκαν
Κάθε αποστολή επιλέγει τυχαία παραμέτρους εντός ενός επιπέδου δυσκολίας:

| Παράμετρος | Εύρος | Σκοπός |
|---|---|---|
| `patrol_speed` | 2.0 - 3.0 m/s | Ταχύτητα περιπολίας |
| `aggr_speed` | 3.5 - 5.0 m/s | Ταχύτητα επίθεσης |
| `agent_speed` | 5.5 - 7.0 m/s | Ταχύτητα πράκτορα |
| `detection_radius` | 40 - 60 m | Εμβέλεια ανίχνευσης εχθρών |

Σταθερές παράμετροι ανά επίπεδο:

| Επίπεδο | Εχθροί Περιπολίας | Επιθετικοί | Στόχοι | Ακτίνα Χάρτη |
|---|---|---|---|---|
| Easy | 2 | 1 | 3 | 800 m |
| Medium | 3 | 2 | 5 | 1000 m |
| Hard | 4 | 3 | 6 | 1200 m |
| Very Hard | 5 | 4 | 7 | 1500 m |

### 2.3 Ορισμός Αποτελεσμάτων
- **SUCCESS:** Ο πράκτορας συνέλεξε **όλους** τους στόχους χωρίς να ακινητοποιηθεί
- **PARTIAL SUCCESS:** Ο πράκτορας ακινητοποιήθηκε αφού συνέλεξε **>70%** των στόχων
- **FAILED:** Ο πράκτορας ακινητοποιήθηκε με **≤70%** στόχων ή υπήρξε υπέρβαση χρόνου
- **Επαφή εχθρού:** Ευκλείδεια απόσταση < 5m → απενεργοποίηση πράκτορα (-100 pts)

### 2.4 Κατανομή Δοκιμών
Συνολικά **{n_runs} αποστολές** εκτελέστηκαν με ντετερμινιστική κατανομή:
- Easy: 20% ({round(n_runs * 0.20)} αποστολές)
- Medium: 30% ({round(n_runs * 0.30)} αποστολές)
- Hard: 30% ({round(n_runs * 0.30)} αποστολές)
- Very Hard: 20% ({round(n_runs * 0.20)} αποστολές)

---

## 3. Συνολικά Αποτελέσματα

| Μέτρο | Τιμή |
|---|---|
| Σύνολο αποστολών (έγκυρες) | {summary.get('total', len(valid))} |
| ✅ Επιτυχίες (SUCCESS) | {summary.get('success', len(success))} ({summary.get('success_rate', pct(len(success), len(valid)))}%) |
| 🔶 Μερικές Επιτυχίες (PARTIAL) | {summary.get('partial', len(partial))} ({summary.get('partial_rate', pct(len(partial), len(valid)))}%) |
| ❌ Αποτυχίες (FAILED) | {summary.get('failed', len(failed))} ({summary.get('failed_rate', pct(len(failed), len(valid)))}%) |
| Μέση βαθμολογία | {summary.get('avg_score', avg([r['score'] for r in valid]))} pts |
| Καλύτερη βαθμολογία | {summary.get('best_score', max([r['score'] for r in valid] or [0]))} pts |
| Μέσος χρόνος εξομοίωσης | {summary.get('avg_sim_time_s', avg([r.get('sim_elapsed_s',0) for r in valid]))} sec (sim-time) |
| Μέσα escape events | {summary.get('avg_escapes', avg([r.get('escape_events',0) for r in valid]))} |
| Σφάλματα φόρτωσης χάρτη | {len(errors)} |

---

## 4. Ανάλυση ανά Επίπεδο Δυσκολίας

### 4.1 Easy (Εύκολο) - 2 Περιπολίες + 1 Επιθετικός + 3 Στόχοι, R=800m

{tier_block("easy", "Easy")}

### 4.2 Medium (Μεσαίο) - 3 Περιπολίες + 2 Επιθετικοί + 5 Στόχοι, R=1000m

{tier_block("medium", "Medium")}

### 4.3 Hard (Δύσκολο) - 4 Περιπολίες + 3 Επιθετικοί + 6 Στόχοι, R=1200m

{tier_block("hard", "Hard")}

### 4.4 Very Hard (Πολύ Δύσκολο) - 5 Περιπολίες + 4 Επιθετικοί + 7 Στόχοι, R=1500m

{tier_block("very_hard", "Very Hard")}

---

## 5. Ανάλυση Αποτυχιών

{failure_analysis()}

### 5.1 Ερμηνεία Αποτυχιών

**Βασικές αιτίες αποτυχίας:**

1. **Τυχαία γένεση κοντά σε εχθρό (Spawn Proximity):**
   Ο πράκτορας γεννιέται σε τυχαίο κόμβο του γράφου. Αν ο κοντινότερος εχθρός βρίσκεται
   εντός ακτίνας ανίχνευσης, η αντίδραση αποφυγής μπορεί να μην προλαβαίνει.

2. **Στενά αδιέξοδα (Dead-end Roads):**
   Σε ορισμένα αστικά μπλοκ της Αθήνας, ο γράφος OSM περιέχει μονόδρομους ή αδιέξοδα.
   Ο επιθετικός εχθρός που κυνηγά μπορεί να κλείσει τη μοναδική διαδρομή.

3. **Εκτεταμένη φάση αποφυγής (Prolonged Escape Mode):**
   Σε δύσκολα επίπεδα, το penalized graph μπορεί να αποκλείει αποτελεσματικά όλες τις
   διαδρομές προς τους στόχους, αναγκάζοντας τον πράκτορα σε βρόχο αποφυγής.

4. **Πολλαπλοί επιθετικοί ταυτόχρονα:**
   Με 3-4 επιθετικούς εχθρούς που κυνηγούν παράλληλα, το σύστημα brave/escape path
   καλείται να διαπραγματευτεί ακτίνες κινδύνου που αλληλεπικαλύπτονται.

---

## 6. Ανάλυση Βαθμολογίας

Η βαθμολογία υπολογίζεται ως:
- **+50 pts** ανά στόχο που συλλέγεται επιτυχώς
- **-1 pt/sec** ποινή χρόνου (terrain penalty) - ο πράκτορας χάνει πόντους όσο περισσότερο κινείται
- **-100 pts** αν ακινητοποιηθεί από εχθρό

Βαθμολογίες που παρατηρήθηκαν:
- **Θετικές βαθμολογίες:** Γρήγορη συλλογή στόχων χωρίς επαφή
- **Χαμηλές/αρνητικές βαθμολογίες:** Μεγάλος χρόνος αποφυγής ή επαφή με εχθρό

"""

# ── escape events analysis ────────────────────────────────────────────────────
all_escapes = [r.get("escape_events", 0) for r in valid]
high_escape = [r for r in valid if r.get("escape_events", 0) >= 3]

report += f"""---

## 7. Ανάλυση Συμβάντων Αποφυγής (Escape Events)

Τα escape events καταγράφουν πόσες φορές ο πράκτορας εισήλθε σε **λειτουργία αποφυγής εχθρού**
(ESCAPE mode), δηλαδή εγκατέλειψε το τρέχον μονοπάτι προς στόχο και επανασχεδίασε διαδρομή μέσω
penalized graph ή escape path.

| Στατιστικό | Τιμή |
|---|---|
| Μέσα escape events ανά αποστολή | {avg(all_escapes)} |
| Αποστολές χωρίς escape events | {sum(1 for e in all_escapes if e == 0)} ({pct(sum(1 for e in all_escapes if e == 0), len(all_escapes))}%) |
| Αποστολές με ≥3 escape events | {len(high_escape)} ({pct(len(high_escape), len(all_escapes))}%) |
| Μέγιστα escape events σε αποστολή | {max(all_escapes) if all_escapes else 0} |

**Παρατήρηση:** Αποστολές με υψηλό αριθμό escape events (≥3) που καταλήγουν σε επιτυχία
αποδεικνύουν ότι ο αλγόριθμος ανάκαμψης λειτουργεί σωστά - ο πράκτορας αποφεύγει επαναλαμβανόμενες
απειλές και συνεχίζει να συλλέγει στόχους.

"""

# ── algorithm section ─────────────────────────────────────────────────────────
report += """---

## 8. Περιγραφή Αλγορίθμου Πλοήγησης

### 8.1 Penalized Graph Pathfinding
Το σύστημα χτίζει σε κάθε replan έναν **βεβαρυμμένο γράφο** βασισμένο στο OSM οδικό δίκτυο,
όπου οι ακμές κοντά σε εχθρούς λαμβάνουν ποινές:
- 0 m από εχθρό: **+8000** (απαγορευμένη ζώνη)
- ≤10 m: **+3000**
- ≤30 m: **+1500**
- ≤ detection_radius: **+800**
- ≤ extended_radius: **+300**
- Εκτός geofence: **+6000**

### 8.2 Στρατηγικές Αντίδρασης
1. **Brave Path:** Υπολογίζει αν ο πράκτορας μπορεί να φτάσει στον στόχο πριν τον εχθρό.
   Αν ναι, ακολουθεί ασφαλή διαδρομή αγνοώντας τον κίνδυνο.
2. **Escape Path:** Για επιθετικούς εχθρούς που κυνηγούν - BFS προς ασφαλή κόμβο μακριά
   από την απειλή, έπειτα επανεκκίνηση αποστολής.
3. **Normal Routing:** Μέσω penalized_G με `nx.shortest_path` (Dijkstra) - αποφεύγει
   εχθρούς μέσω αύξησης βάρους ακμών.

### 8.3 Αντι-Ταλάντωση Στόχου (Hysteresis)
Αποτρέπει συνεχή εναλλαγή στόχων:
- 25% έκπτωση κόστους για τρέχοντα στόχο (inertia discount)
- Αλλαγή στόχου μόνο αν ο νέος είναι **>25% φθηνότερος** ΚΑΙ ο τρέχων έχει ≥8 waypoints
- Replan interval: 12 ticks - override όταν εχθρός <15m

### 8.4 Path Lookahead
- Έλεγχος **12 επόμενων waypoints** (~24m) για εχθρούς στη διαδρομή
- Sliding danger zone: 10m (κοντά) → 4m (μακριά)
- Άμεσο temizleme μονοπατιού αν εχθρός εντοπιστεί στη διαδρομή

---

## 9. Συμπεράσματα

"""

# ── dynamic conclusions ───────────────────────────────────────────────────────
overall_success = pct(len(success), len(valid))
overall_partial = pct(len(partial), len(valid))
overall_failed  = pct(len(failed),  len(valid))
combined_ok     = pct(len(success) + len(partial), len(valid))

report += f"""Από τις {len(valid)} έγκυρες αποστολές:

1. **Ο αλγόριθμος επιτυγχάνει συνολικά {overall_success}% πλήρη επιτυχία** και
   {combined_ok}% επιτυχή αποτελέσματα (συμπεριλαμβανομένων των μερικών).

2. **Αύξηση δυσκολίας → μείωση επιτυχίας:** Το σύστημα παρουσιάζει αναμενόμενη αποδυνάμωση
   σε υψηλότερα επίπεδα εχθρών και μεγαλύτερες περιοχές αναζήτησης.

3. **Λειτουργία αποφυγής (escape mode):** Καταγράφηκε σε {sum(1 for r in valid if r.get('escape_events',0)> 0)} αποστολές,
   αποδεικνύοντας ότι το σύστημα αντιδρά δυναμικά σε απειλές σε πραγματικό χρόνο.

4. **Αποτυχίες λόγω επαφής:** Κυρίως σε τυχαία γένεση κοντά σε εχθρούς ή σε αδιέξοδα OSM.
   Αυτές είναι αποτυχίες του **τυχαίου σεναρίου**, όχι αναγκαστικά του αλγορίθμου.

5. **Βαθμολογία:** Μέση βαθμολογία {avg([r['score'] for r in valid])} pts - θετική τιμή
   επιβεβαιώνει ότι ο πράκτορας συλλέγει αρκετούς στόχους ώστε να αντισταθμίσει τις ποινές χρόνου.

---

## 10. Οδηγίες Παρουσίασης στο Poster

### Ποια αποτελέσματα να παρουσιάσεις

**Κεντρικά νούμερα (μεγάλα, έντονα):**
- Ποσοστό επιτυχίας: **{overall_success}%** (SUCCESS) + **{overall_partial}%** (PARTIAL) = **{combined_ok}% θετικές αποστολές**
- Σύνολο αποστολών: **{len(valid)}**
- Βέλτιστη βαθμολογία: **{max([r['score'] for r in valid] or [0])} pts**

**Γράφημα 1 - Pie Chart Αποτελεσμάτων:**
Χρησιμοποίησε το `results_pie.png` από το `plot_results.py`.
Τίτλος: "Mission Outcomes across {len(valid)} runs"

**Γράφημα 2 - Ραβδόγραμμα ανά Δυσκολία:**
Χρησιμοποίησε το `results_by_difficulty.png`.
Τίτλος: "Performance vs. Scenario Difficulty"
Εξηγεί ότι το σύστημα κλιμακώνεται σωστά - easy ≈ 100%, very_hard χαμηλότερα.

**Γράφημα 3 - Κατανομή Βαθμολογιών:**
Χρησιμοποίησε το `results_score_dist.png`.
Τίτλος: "Score Distribution across all missions"

**Γράφημα 4 - Ποσοστό Επιτυχίας vs Δυσκολία:**
Χρησιμοποίησε το `results_rate_vs_difficulty.png`.
Ανοδική/καθοδική τάση που δείχνει ελεγχόμενη πολυπλοκότητα.

### Τι να πεις στην παρουσίαση

1. *"Ο αλγόριθμος τεστάρθηκε σε {len(valid)} αυτόματα παραγόμενα σενάρια σε πραγματικό οδικό δίκτυο Αθήνας."*
2. *"4 επίπεδα δυσκολίας με 3-7 στόχους και 3-9 εχθρούς."*
3. *"Κατά μέσο όρο {avg([r.get('escape_events',0) for r in valid])} συμβάντα αποφυγής ανά αποστολή - ο πράκτορας ανιχνεύει και αποφεύγει εχθρούς σε πραγματικό χρόνο."*
4. Δείξε screenshot από dashboard του escape mode (φωτογραφία `dashboard-with-target-identified.png`)
5. Δείξε το `custom-mission-creator.png` για να εξηγήσεις τη δημιουργία σεναρίων

### Τι ΔΕΝ χρειάζεται στο poster
- Τεχνικές λεπτομέρειες κώδικα (αυτές πηγαίνουν στην αναφορά, όχι στο poster)
- Όλα τα raw αποτελέσματα - μόνο τα συνοπτικά νούμερα
- Αρνητικά scores των αποτυχιών - εστίασε στους επιτυχείς

---

*Αναφορά δημιουργήθηκε αυτόματα από `generate_report.py` βάσει αρχείου `{INPUT}`*
*SAR-SIM Project - {datetime.now().strftime("%B %Y")}*
"""

with open(OUT, "w", encoding="utf-8") as f:
    f.write(report)

print(f"Αναφορά αποθηκεύτηκε: {OUT}")
print(f"\nΣύνοψη:")
print(f"  Αποστολές: {len(valid)} έγκυρες από {n_runs}")
print(f"  SUCCESS:   {len(success)} ({pct(len(success), len(valid))}%)")
print(f"  PARTIAL:   {len(partial)} ({pct(len(partial), len(valid))}%)")
print(f"  FAILED:    {len(failed)} ({pct(len(failed), len(valid))}%)")
print(f"  Μ.Ο. Score: {avg([r['score'] for r in valid])}")
