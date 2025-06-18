using UnityEngine;
using RosSharp.RosBridgeClient;
using TMPro;
using Instructions = RosSharp.RosBridgeClient.MessageTypes.Instructions;
using System.Collections.Generic;

public class InstructionsService : UnityServiceProvider<Instructions.InstructionsRequest, Instructions.InstructionsResponse>
{
    [SerializeField] private CanvasHandler _canvas;
    private Queue<Instructions.InstructionsRequest> requestsQueue = new Queue<Instructions.InstructionsRequest>();

    void Start()
    {
        Debug.Log("InstructionsService: Start chamado e inicializado.");
        base.Start(); 
        if (_canvas == null)
        {
            Debug.LogWarning("InstructionsService: _canvas É NULL no Start.");
        }
        else
        {
            Debug.Log("InstructionsService: _canvas NÃO É NULL no Start.");
        }
    }

    protected override bool ServiceCallHandler(Instructions.InstructionsRequest request, out Instructions.InstructionsResponse response)
    {
        Debug.Log("[InstructionsService] ServiceCallHandler: INÍCIO DA CHAMADA.");
        Debug.Log("[InstructionsService] Instrução recebida: " + request.instruction);

        requestsQueue.Enqueue(request);

        response = new Instructions.InstructionsResponse
        {
            success = true
        };
        Debug.Log("[InstructionsService] Requisição enfileirada. Resposta de sucesso preparada. FIM DA CHAMADA.");
        return true;
    }

    private void Update()
    {
        while (requestsQueue.Count > 0)
        {
            var request = requestsQueue.Dequeue();
            ProcessInstructionRequest(request);
        }
    }

    private void ProcessInstructionRequest(Instructions.InstructionsRequest request)
    {
        Debug.Log("[InstructionsService] ProcessInstructionRequest: Processando instrução: " + request.instruction);
        if (_canvas != null)
        {
            Debug.Log("[InstructionsService] _canvas NÃO é null. Tentando chamar UpdateInstruction.");
            _canvas.UpdateInstruction(request.instruction);
            Debug.Log("[InstructionsService] UpdateInstruction chamado. Instrução enviada: " + request.instruction);
        }
        else
        {
            Debug.LogWarning("[InstructionsService] _canvas É null em ProcessInstructionRequest. Não é possível chamar UpdateInstruction.");
        }
    }

    // Novo método público para ser chamado por outros scripts Unity
    public void DisplayInstruction(string instructionText)
    {
        Debug.Log("[InstructionService] DisplayInstruction chamada diretamente com: " + instructionText);
        if (_canvas != null)
        {
            _canvas.UpdateInstruction(instructionText);
            Debug.Log("[InstructionService] _canvas.UpdateInstruction chamada a partir de DisplayInstruction.");
        }
        else
        {
            Debug.LogWarning("[InstructionService] _canvas é null ao chamar DisplayInstruction.");
        }
    }
}