"""Run in WIR3D's torch environment: verify chunking preserves Adam updates."""

import torch

for batch in (1, 4):
    torch.manual_seed(42)
    full = torch.zeros(37, 11, requires_grad=True)
    chunks = [
        torch.zeros(min(8, 37 - start), 11, requires_grad=True) for start in range(0, 37, 8)
    ]
    reference = torch.optim.Adam([full], lr=0.1, foreach=False)
    chunked = torch.optim.Adam(chunks, lr=0.1, foreach=False)
    for _step in range(7):
        features = torch.randn(batch, 37, 11)
        visible = torch.rand(batch, 37) > 0.4
        reference.zero_grad()
        chunked.zero_grad()
        ((features - full * visible.unsqueeze(-1)) ** 2).sum().backward()
        for index, chunk in enumerate(chunks):
            section = slice(index * 8, index * 8 + len(chunk))
            (
                (features[:, section] - chunk * visible[:, section].unsqueeze(-1)) ** 2
            ).sum().backward()
        reference.step()
        chunked.step()
    torch.testing.assert_close(full, torch.cat(chunks), rtol=0, atol=0)
    print(f"Exact Adam equivalence passed, batch {batch}")
