"""Render bom.csv (bought parts) and ../exports/printed_parts.csv into BOM.md."""
import csv
from pathlib import Path

HERE = Path(__file__).parent


def main():
    rows = list(csv.DictReader(open(HERE / "bom.csv")))
    out = ["# GRIP bill of materials", "",
           "Bought parts for version B. Prices marked *estimate* could not be read from the store page "
           "(Amazon and AliExpress block automated reads); check them when ordering. "
           "Printed parts are listed after, with the bureau quote noted separately.", ""]
    groups = {}
    for r in rows:
        groups.setdefault(r["group"], []).append(r)
    grand = 0.0
    for g, items in groups.items():
        out += [f"## {g}", "", "| Item | Spec | Qty | Unit (USD) | Line (USD) | Price | Buy |",
                "| --- | --- | ---: | ---: | ---: | --- | --- |"]
        sub = 0.0
        for r in items:
            line = float(r["qty"]) * float(r["unit_usd"])
            sub += line
            out.append(f"| {r['item']} | {r['spec']} | {r['qty']} | {float(r['unit_usd']):.2f} | {line:.2f} | "
                       f"{r['price_status']} | [{r['source']}]({r['url']}) |")
        out += ["", f"Subtotal: **${sub:,.2f}**", ""]
        grand += sub
    out += [f"**Bought parts total: ${grand:,.2f}**", ""]
    pp = HERE.parent / "exports" / "printed_parts.csv"
    if pp.exists():
        prows = list(csv.DictReader(open(pp)))
        vol = sum(float(r["volume_cm3"]) * int(r["qty"]) for r in prows)
        out += ["## Printed parts", "",
                f"{len(prows)} unique parts, {sum(int(r['qty']) for r in prows)} pieces, {vol:,.0f} cm³ in total. "
                "Full list with materials in [exports/printed_parts.csv](../exports/printed_parts.csv); "
                "bureau stacked kits in [exports/service_kits.csv](../exports/service_kits.csv).", ""]
    (HERE / "BOM.md").write_text("\n".join(out) + "\n")
    print(f"bought parts ${grand:,.2f}")


if __name__ == "__main__":
    main()
