# Section 1
# Convert csv file into line items and queries
# This code assumes a file format similar to the one on the
# Arduino BOM
import csv


csv_file = open("bom.csv", "r")
csv_reader = csv.DictReader(csv_file)
line_items = []
queries = []
for line_item in csv_reader:
    # Skip line items without part numbers and manufacturers
    if not line_item['Part Number'] or not line_item['Manufacturer']:
        continue
    line_items.append(line_item)
    queries.append({'mpn': line_item['Part Number'],
                    'brand': line_item['Manufacturer'],
                    'reference': len(line_items) - 1})


# Section 2
# Send queries to REST API for part matching.
import json
import urllib


results = []
for i in range(0, len(queries), 20):
    # Batch queries in groups of 20, query limit of
    # parts match endpoint
    batched_queries = queries[i: i + 20]

    url = 'http://octopart.com/api/v3/parts/match?queries=%s' \
        % urllib.quote(json.dumps(batched_queries))
    url += '&apikey=93cecea3'
    data = urllib.urlopen(url).read()
    response = json.loads(data)

    # Record results for analysis
    results.extend(response['results'])


# Section 3
# Analyze results sent back by Octopart API
from decimal import Decimal


print "Found %s line items in BOM." % len(line_items)
# Price BOM
hits = 0
total_avg_price = 0
for result in results:
    line_item = line_items[result['reference']]
    if len(result['items']) == 0:
        print "Did not find match on line item %s" % line_item
        continue

    # Get pricing from the first item for desired quantity
    quantity = Decimal(line_items[result['reference']]['Qty'])
    prices = []
    for offer in result['items'][0]['offers']:
        if 'USD' not in offer['prices'].keys():
            continue
        price = None
        for price_tuple in offer['prices']['USD']:
            # Find correct price break
            if price_tuple[0] > quantity:
                break
            # Cast pricing string to Decimal for precision
            # calculations
            price = Decimal(price_tuple[1])
        if price is not None:
            prices.append(price)

    if len(prices) == 0:
        print "Did not find pricing on line item %s" % line_item
        continue
    avg_price = quantity * sum(prices) / len(prices)
    total_avg_price += avg_price
    hits += 1

print "Matched on %.2f of BOM, total average price is USD %.2f" % ( \
    hits / float(len(line_items)), total_avg_price)
